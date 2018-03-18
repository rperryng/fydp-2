#include "stdafx.h"
#include "BodyLandmarkRecognizer.h"

using namespace cv;
using std::vector;

BodyLandmarkRecognizer::BodyLandmarkRecognizer(
	UINT16 *depthBuffer,
	int depthBufferHeight,
	int depthBufferWidth,
	RGBQUAD *colorBuffer,
	int colorBufferHeight,
	int colorBufferWidth,
	Joint *joints,
	ICoordinateMapper *coordinateMapper
) {
	m_depthBuffer = depthBuffer;
	m_depthBufferHeight = depthBufferHeight;
	m_depthBufferWidth = depthBufferWidth;

	m_colorBuffer = colorBuffer;
	m_colorBufferHeight = colorBufferHeight;
	m_colorBufferWidth = colorBufferWidth;

	m_joints = joints;

	m_coordinateMapper = coordinateMapper;

	// Convert all joints into depth space
	for (int i = 0; i < JointType_Count; i++) {
		Point depthPoint = JointToDepthSpace((JointType) i);
		m_jointsDepthSpace[i] = depthPoint;
	}

	m_matDepthRaw = Mat(m_depthBufferHeight, m_depthBufferWidth, CV_16UC1, m_depthBuffer, sizeof(UINT16) * m_depthBufferWidth);
	cvtColor(m_matDepthRaw, m_matDepth, COLOR_GRAY2BGR);
	m_matColor = Mat(m_colorBufferHeight, m_colorBufferWidth, CV_8UC4, m_colorBuffer).clone();
}

Point BodyLandmarkRecognizer::GetOffsetForJoint(Joint joint) {
	ColorSpacePoint csp_control = { 0 };
	ColorSpacePoint csp = { 0 };
	DepthSpacePoint dsp = { 0 };

	m_coordinateMapper->MapCameraPointToColorSpace(joint.Position, &csp_control);
	m_coordinateMapper->MapCameraPointToDepthSpace(joint.Position, &dsp);

	UINT16 depthValue = m_depthBuffer[((int) dsp.Y) * m_depthBufferWidth + ((int) dsp.X)];
	m_coordinateMapper->MapDepthPointToColorSpace(dsp, depthValue, &csp);
	return Point(csp_control.X - csp.X, csp_control.Y - csp.Y);
}


vector<Point> BodyLandmarkRecognizer::recognizeFor(ClothingType clothingType) {
	switch (clothingType)
	{
	case ClothingType_Shirt:
		return recognizeForShirt();
	case ClothingType_Shorts:
		return recognizeForShorts();
	default:
		throw std::invalid_argument("Clothing type received not recognized");
	}
}

Point BodyLandmarkRecognizer::findBoundary(Mat matDepth, Point start, bool traverseRight, float slope) {
	if (traverseRight) {
		for (int x = start.x; x < m_depthBufferWidth; x++) {
			int y = start.y + (slope * (x - start.x));

			if (matDepth.at<USHORT>(y, x) == 0) {
				return Point(x, y);
			}
		}
	} else {
		for (int x = start.x; x >= 0; x--) {
			int y = start.y + (slope * (x - start.x));

			if (matDepth.at<USHORT>(y, x) == 0) {
				return Point(x, y);
			}
		}
	}

	return Point(0, 0);
}

Point BodyLandmarkRecognizer::JointToDepthSpace(JointType jointType) {
	DepthSpacePoint dsp = { 0 };
	Joint joint = m_joints[jointType];
	m_coordinateMapper->MapCameraPointToDepthSpace(joint.Position, &dsp);
	return Point((int) dsp.X, (int) dsp.Y);
}

Point BodyLandmarkRecognizer::DepthSpaceToColorSpace(Point point) {
	DepthSpacePoint dsp = { 0 };
	ColorSpacePoint csp = { 0 };
	dsp.X = (float) point.x;
	dsp.Y = (float) point.y;
	m_coordinateMapper->MapDepthPointToColorSpace(dsp, USHRT_MAX, &csp);
	return Point(csp.X, csp.Y);
}

float calculateSlope(Point p1, Point p2) {
	return ((float) (p1.x - p2.x)) / (p1.y - p2.y);
}

Point pointAverage(Point p1, Point p2) {
	return Point(
		(p1.x + p2.x) / 2,
		(p1.y + p2.y) / 2
	);
}

std::vector<Point> BodyLandmarkRecognizer::recognizeForShirt() {
	vector<Point> pointsShirt(12);
	Point csp;
	Point offset;
	Point delta;
	float slope;

	// Neck
	m_tracePointsShirt[TPS_LeftNeck] = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_Neck], false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_LeftNeck]);
	offset = GetOffsetForJoint(m_joints[JointType_Neck]);
	pointsShirt[0] = csp + offset;

	m_tracePointsShirt[TPS_RightNeck] = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_Neck], true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_RightNeck]);
	pointsShirt[1] = csp + offset;

	// Shoulder Left
	m_tracePointsShirt[TPS_LeftShoulder] = findBoundary(
		m_matDepthRaw,
		m_jointsDepthSpace[JointType_ShoulderLeft],
		false,
		1.0f
	);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_LeftShoulder]);
	offset = GetOffsetForJoint(m_joints[JointType_ShoulderLeft]);
	pointsShirt[2] = csp + offset;

	// Right Shoulder
	m_tracePointsShirt[TPS_RightShoulder] = findBoundary(
		m_matDepthRaw,
		m_jointsDepthSpace[JointType_ShoulderRight],
		true,
		-1.0f
	);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_RightShoulder]);
	offset = GetOffsetForJoint(m_joints[JointType_ShoulderRight]);
	pointsShirt[3] = csp + offset;

	// Left Hip
	m_tracePointsShirt[TPS_LeftHip] = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_HipLeft], false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_LeftHip]);
	offset = GetOffsetForJoint(m_joints[JointType_HipLeft]);
	pointsShirt[10] = csp + offset;

	// Right Hip
	m_tracePointsShirt[TPS_RightHip] = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_HipRight], true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_RightHip]);
	offset = GetOffsetForJoint(m_joints[JointType_HipRight]);
	pointsShirt[11] = csp + offset;


	// Left Hem
	Point pointLeftElbow = m_jointsDepthSpace[JointType_ElbowLeft];
	Point pointLeftShoulder = m_jointsDepthSpace[JointType_ShoulderLeft];
	Point leftBicep = pointAverage(pointLeftElbow, pointLeftShoulder);
	slope = -((float) (pointLeftElbow.x - pointLeftShoulder.x)) / (pointLeftElbow.y - pointLeftShoulder.y);
	m_tracePointsShirt[TPS_LeftOuterHem] = findBoundary(m_matDepthRaw, leftBicep, false, slope);
	delta = leftBicep - m_tracePointsShirt[TPS_LeftOuterHem];
	m_tracePointsShirt[TPS_LeftInnerHem] = leftBicep + delta;

	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_LeftOuterHem]);
	offset = GetOffsetForJoint(m_joints[JointType_ElbowLeft]);
	pointsShirt[4] = csp + offset;

	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_LeftInnerHem]);
	pointsShirt[6] = csp + offset;

	// Right Hem
	Point rightElbow = m_jointsDepthSpace[JointType_ElbowRight];
	Point rightShoulder = m_jointsDepthSpace[JointType_ShoulderRight];
	Point rightBicep = pointAverage(rightElbow, rightShoulder);
	slope = -((float) (rightElbow.x - rightShoulder.x)) / (rightElbow.y - rightShoulder.y);
	m_tracePointsShirt[TPS_RightOuterHem] = findBoundary(m_matDepthRaw, rightBicep, true, slope);
	delta = rightBicep - m_tracePointsShirt[TPS_RightOuterHem];
	m_tracePointsShirt[TPS_RightInnerHem] = rightBicep + delta;

	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_RightOuterHem]);
	offset = GetOffsetForJoint(m_joints[JointType_ElbowRight]);
	pointsShirt[5] = csp + offset;

	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_RightInnerHem]);
	pointsShirt[7] = csp + offset;

	// Extra shirt points
	// Left Hip
	Point pointHipLeft = m_jointsDepthSpace[JointType_HipLeft];
	pointHipLeft.x += 5;
	pointHipLeft.y -= 20;
	m_tracePointsShirt[TPS_LeftHipUpper] = findBoundary(m_matDepthRaw, pointHipLeft, false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_LeftHipUpper]);
	offset = GetOffsetForJoint(m_joints[JointType_HipLeft]);
	pointsShirt[8] = csp + offset;

	// Right Hip
	Point pointHipRight = m_jointsDepthSpace[JointType_HipRight];
	pointHipRight.x -= 5;
	pointHipRight.y -= 20;
	m_tracePointsShirt[TPS_RightHipUpper] = findBoundary(m_matDepthRaw, pointHipRight, true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShirt[TPS_RightHipUpper]);
	offset = GetOffsetForJoint(m_joints[JointType_HipRight]);
	pointsShirt[9] = csp + offset;

	for (int i = 0; i < pointsShirt.size(); i++) {
		circle(m_matColor, pointsShirt[i], 5, BLUE, FILLED, LINE_8);
		circle(m_matDepth, m_tracePointsShirt[(TracePointsShirt) i], 5, BLUE_8U, FILLED, LINE_8);
	}

	namedWindow("Connected Components", WINDOW_NORMAL);
	namedWindow("Connected Components with Landmarks", WINDOW_NORMAL);
	namedWindow("Color with Landmarks", WINDOW_NORMAL);

	imshow("Connected Components", m_matDepthRaw);
	imshow("Connected Components with Landmarks", m_matDepth);
	imshow("Color with Landmarks", m_matColor);

	waitKey(0);

	return pointsShirt;
}

vector<Point> BodyLandmarkRecognizer::recognizeForShorts() {
	vector<Point> pointsPants(9);
	Point csp;
	Point offset;
	Point delta;
	float slope;

	// Left Hip
	Point pointHipLeft = m_jointsDepthSpace[JointType_HipLeft];
	pointHipLeft.x += 5;
	pointHipLeft.y -= 20;
	m_tracePointsShorts[TPSH_LeftHip] = findBoundary(m_matDepthRaw, pointHipLeft, false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_LeftHip]);
	offset = GetOffsetForJoint(m_joints[JointType_HipLeft]);
	pointsPants[0] = csp + offset;

	// Right Hip
	Point pointHipRight = m_jointsDepthSpace[JointType_HipRight];
	pointHipRight.x -= 5;
	pointHipRight.y -= 20;
	m_tracePointsShorts[TPSH_RightHip] = findBoundary(m_matDepthRaw, pointHipRight, true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_RightHip]);
	offset = GetOffsetForJoint(m_joints[JointType_HipRight]);
	pointsPants[1] = csp + offset;

	// Crotch
	Point pointCrotch = m_jointsDepthSpace[JointType_SpineBase];
	for (int y = pointCrotch.y; y < m_depthBufferHeight; y++) {
		if (m_matDepthRaw.at<USHORT>(y, pointCrotch.x) == 0) {
			m_tracePointsShorts[TPSH_Crotch] = Point(pointCrotch.x, y);
			break;
		}
	}

	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_Crotch]);
	offset = GetOffsetForJoint(m_joints[JointType_SpineBase]);
	pointsPants[3] = csp + offset;

	// Left Knee
	Point pointLeftHip = m_jointsDepthSpace[JointType_HipLeft];
	Point pointLeftKnee = m_jointsDepthSpace[JointType_KneeLeft];
	slope = -((float) (pointLeftHip.x - pointLeftKnee.x)) /  (pointLeftHip.y - pointLeftKnee.y);
	m_tracePointsShorts[TPSH_LeftOuterKnee] = findBoundary(m_matDepthRaw, pointLeftKnee, false, slope);
	m_tracePointsShorts[TPSH_LeftInnerKnee] = findBoundary(m_matDepthRaw, pointLeftKnee, true, slope);
	
	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_LeftOuterKnee]);
	offset = GetOffsetForJoint(m_joints[JointType_KneeLeft]);
	pointsPants[5] = csp + offset;

	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_LeftInnerKnee]);
	offset = GetOffsetForJoint(m_joints[JointType_KneeLeft]);
	pointsPants[6] = csp + offset;

	// Right Knee
	Point pointRightHip = m_jointsDepthSpace[JointType_HipRight];
	Point pointRightKnee = m_jointsDepthSpace[JointType_KneeRight];
	slope = -((float)(pointRightHip.x - pointRightKnee.x)) / (pointRightHip.y - pointRightKnee.y);
	m_tracePointsShorts[TPSH_RightOuterKnee] = findBoundary(m_matDepthRaw, pointRightKnee, true, slope);
	m_tracePointsShorts[TPSH_RightInnerKnee] = findBoundary(m_matDepthRaw, pointRightKnee, false, slope);
	
	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_RightOuterKnee]);
	offset = GetOffsetForJoint(m_joints[JointType_KneeRight]);
	pointsPants[8] = csp + offset;

	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_RightInnerKnee]);
	offset = GetOffsetForJoint(m_joints[JointType_KneeRight]);
	pointsPants[7] = csp + offset;

	// Left Quad
	Point quadLeft = pointAverage(pointLeftHip, pointLeftKnee);
	m_tracePointsShorts[TPSH_LeftOuterQuad] = findBoundary(m_matDepthRaw, quadLeft, false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_LeftOuterQuad]);
	offset = GetOffsetForJoint(m_joints[JointType_KneeLeft]);
	pointsPants[2] = csp + offset;

	// Right Quad
	Point quadRight = pointAverage(pointRightHip, pointRightKnee);
	m_tracePointsShorts[TPSH_RightOuterQuad] = findBoundary(m_matDepthRaw, quadRight, true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePointsShorts[TPSH_RightOuterQuad]);
	offset = GetOffsetForJoint(m_joints[JointType_KneeRight]);
	pointsPants[4] = csp + offset;

	for (int i = 0; i < pointsPants.size(); i++) {
		circle(m_matColor, pointsPants[i], 5, GREEN, FILLED, LINE_8);
		circle(m_matDepth, m_tracePointsShorts[(TracePointsShorts) i], 5, GREEN_8U, FILLED, LINE_8);
	}

	namedWindow("Connected Components", WINDOW_NORMAL);
	namedWindow("Connected Components with Landmarks", WINDOW_NORMAL);
	namedWindow("Color with Landmarks", WINDOW_NORMAL);

	imshow("Connected Components", m_matDepthRaw);
	imshow("Connected Components with Landmarks", m_matDepth);
	imshow("Color with Landmarks", m_matColor);

	waitKey(0);

	return pointsPants;
}
