#include "stdafx.h"
#include "BodyLandmarkRecognizer.h"

using namespace cv;
using std::vector;

#define RED_8U Scalar(0, 0, USHRT_MAX)
#define BLUE_8U Scalar(USHRT_MAX, 0, 0)
#define GREEN_8U Scalar(0, USHRT_MAX, 0)
#define RED Scalar(0, 0, 255)
#define BLUE Scalar(255, 0, 0)
#define GREEN Scalar(0, 255, 0)

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

ColorSpacePoint BodyLandmarkRecognizer::DepthSpaceToColorSpace(int x, int y) {
	DepthSpacePoint dsp = { 0 };
	ColorSpacePoint csp = { 0 };
	dsp.X = (float) x;
	dsp.Y = (float) y;
	m_coordinateMapper->MapDepthPointToColorSpace(dsp, USHRT_MAX, &csp);
	return csp;
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
	ColorSpacePoint csp;
	DepthSpacePoint dsp;
	Point offset;
	Point delta;
	float slope;

	// Neck
	m_tracePoints.leftNeck = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_Neck], false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftNeck.x, m_tracePoints.leftNeck.y);
	offset = GetOffsetForJoint(m_joints[JointType_Neck]);
	pointsShirt[0] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.leftNeck.x, m_tracePoints.leftNeck.y), 5, BLUE_8U, FILLED, LINE_8);

	m_tracePoints.rightNeck = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_Neck], true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightNeck.x, m_tracePoints.rightNeck.y);
	pointsShirt[1] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.rightNeck.x, m_tracePoints.rightNeck.y), 5, BLUE_8U, FILLED, LINE_8);

	// Shoulder Left
	m_tracePoints.leftShoulder = findBoundary(
		m_matDepthRaw,
		m_jointsDepthSpace[JointType_ShoulderLeft],
		false,
		1.0f
	);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftShoulder.x, m_tracePoints.leftShoulder.y);
	offset = GetOffsetForJoint(m_joints[JointType_ShoulderLeft]);
	pointsShirt[2] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.leftShoulder.x, m_tracePoints.leftShoulder.y), 5, BLUE_8U, FILLED, LINE_8);

	// Right Shoulder
	m_tracePoints.rightShoulder = findBoundary(
		m_matDepthRaw,
		m_jointsDepthSpace[JointType_ShoulderRight],
		true,
		-1.0f
	);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightShoulder.x, m_tracePoints.rightShoulder.y);
	offset = GetOffsetForJoint(m_joints[JointType_ShoulderRight]);
	pointsShirt[3] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.rightShoulder.x, m_tracePoints.rightShoulder.y), 5, BLUE_8U, FILLED, LINE_8);

	// Left Hip
	m_tracePoints.leftHip = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_HipLeft], false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftHip.x, m_tracePoints.leftHip.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipLeft]);
	pointsShirt[10] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.leftHip.x, m_tracePoints.leftHip.y), 5, BLUE_8U, FILLED, LINE_8);

	// Right Hip
	m_tracePoints.rightHip = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_HipRight], true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightHip.x, m_tracePoints.rightHip.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipRight]);
	pointsShirt[11] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.rightHip.x, m_tracePoints.rightHip.y), 5, BLUE_8U, FILLED, LINE_8);


	// Left Hem
	Point pointLeftElbow = m_jointsDepthSpace[JointType_ElbowLeft];
	Point pointLeftShoulder = m_jointsDepthSpace[JointType_ShoulderLeft];
	Point leftBicep = pointAverage(pointLeftElbow, pointLeftShoulder);
	slope = -((float) (pointLeftElbow.x - pointLeftShoulder.x)) / (pointLeftElbow.y - pointLeftShoulder.y);
	m_tracePoints.leftOuterHem = findBoundary(m_matDepthRaw, leftBicep, false, slope);
	delta = leftBicep - m_tracePoints.leftOuterHem;
	m_tracePoints.leftInnerHem = leftBicep + delta;

	csp = DepthSpaceToColorSpace(m_tracePoints.leftOuterHem.x, m_tracePoints.leftOuterHem.y);
	offset = GetOffsetForJoint(m_joints[JointType_ElbowLeft]);
	pointsShirt[4] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.leftOuterHem, 5, BLUE_8U, FILLED, LINE_8);

	csp = DepthSpaceToColorSpace(m_tracePoints.leftInnerHem.x, m_tracePoints.leftInnerHem.y);
	pointsShirt[6] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.leftInnerHem, 5, BLUE_8U, FILLED, LINE_8);

	// Right Hem
	Point rightElbow = m_jointsDepthSpace[JointType_ElbowRight];
	Point rightShoulder = m_jointsDepthSpace[JointType_ShoulderRight];
	Point rightBicep = pointAverage(rightElbow, rightShoulder);
	//slope = -((float) (m_skeletalPoints.rightElbow_x - m_skeletalPoints.rightShoulder_x)) / (m_skeletalPoints.rightElbow_y - m_skeletalPoints.rightShoulder_y);
	slope = -((float) (rightElbow.x - rightShoulder.x)) / (rightElbow.y - rightShoulder.y);
	m_tracePoints.rightOuterHem = findBoundary(m_matDepthRaw, rightBicep, true, slope);
	delta = rightBicep - m_tracePoints.rightOuterHem;
	m_tracePoints.rightInnerHem = rightBicep + delta;

	csp = DepthSpaceToColorSpace(m_tracePoints.rightOuterHem.x, m_tracePoints.rightOuterHem.y);
	offset = GetOffsetForJoint(m_joints[JointType_ElbowRight]);
	pointsShirt[5] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.rightOuterHem.x, m_tracePoints.rightOuterHem.y), 5, BLUE_8U, FILLED, LINE_8);

	csp = DepthSpaceToColorSpace(m_tracePoints.rightInnerHem.x, m_tracePoints.rightInnerHem.y);
	pointsShirt[7] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.rightInnerHem.x, m_tracePoints.rightInnerHem.y), 5, BLUE_8U, FILLED, LINE_8);

	// Extra shirt points
	// Left Hip
	Point pointHipLeft = m_jointsDepthSpace[JointType_HipLeft];
	pointHipLeft.x += 5;
	pointHipLeft.y -= 20;
	m_tracePoints.leftHipUpper = findBoundary(m_matDepthRaw, pointHipLeft, false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftHipUpper.x, m_tracePoints.leftHipUpper.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipLeft]);
	pointsShirt[8] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.leftHipUpper.x, m_tracePoints.leftHipUpper.y), 5, BLUE_8U, FILLED, LINE_8);

	// Right Hip
	Point pointHipRight = m_jointsDepthSpace[JointType_HipRight];
	pointHipRight.x -= 5;
	pointHipRight.y -= 20;
	m_tracePoints.rightHipUpper = findBoundary(m_matDepthRaw, pointHipRight, true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightHipUpper.x, m_tracePoints.rightHipUpper.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipRight]);
	pointsShirt[9] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.rightHipUpper.x, m_tracePoints.rightHipUpper.y), 5, BLUE_8U, FILLED, LINE_8);


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
	ColorSpacePoint csp;
	DepthSpacePoint dsp;
	Point offset;
	Point delta;
	float slope;

	// Left Hip
	Point pointHipLeft = m_jointsDepthSpace[JointType_HipLeft];
	pointHipLeft.x += 5;
	pointHipLeft.y -= 20;
	m_tracePoints.leftHipUpper = findBoundary(m_matDepthRaw, pointHipLeft, false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftHipUpper.x, m_tracePoints.leftHipUpper.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipLeft]);
	pointsPants[0] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.leftHipUpper.x, m_tracePoints.leftHipUpper.y), 5, GREEN_8U, FILLED, LINE_8);

	// Right Hip
	Point pointHipRight = m_jointsDepthSpace[JointType_HipRight];
	pointHipRight.x -= 5;
	pointHipRight.y -= 20;
	m_tracePoints.rightHipUpper = findBoundary(m_matDepthRaw, pointHipRight, true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightHipUpper.x, m_tracePoints.rightHipUpper.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipRight]);
	pointsPants[1] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, Point(m_tracePoints.rightHipUpper.x, m_tracePoints.rightHipUpper.y), 5, GREEN_8U, FILLED, LINE_8);

	// Crotch
	Point pointCrotch = m_jointsDepthSpace[JointType_SpineBase];
	for (int y = pointCrotch.y; y < m_depthBufferHeight; y++) {
		if (m_matDepthRaw.at<USHORT>(y, pointCrotch.x) == 0) {
			m_tracePoints.crotch = Point(pointCrotch.x, y);
			break;
		}
	}

	csp = DepthSpaceToColorSpace(m_tracePoints.crotch.x, m_tracePoints.crotch.y);
	offset = GetOffsetForJoint(m_joints[JointType_SpineBase]);
	pointsPants[3] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.crotch, 5, GREEN_8U, FILLED, LINE_8);

	// Left Knee
	Point pointLeftHip = m_jointsDepthSpace[JointType_HipLeft];
	Point pointLeftKnee = m_jointsDepthSpace[JointType_KneeLeft];
	slope = -((float) (pointLeftHip.x - pointLeftKnee.x)) /  (pointLeftHip.y - pointLeftKnee.y);
	m_tracePoints.leftOuterKnee = findBoundary(m_matDepthRaw, pointLeftKnee, false, slope);
	m_tracePoints.leftInnerKnee = findBoundary(m_matDepthRaw, pointLeftKnee, true, slope);
	
	csp = DepthSpaceToColorSpace(m_tracePoints.leftOuterKnee.x, m_tracePoints.leftOuterKnee.y);
	offset = GetOffsetForJoint(m_joints[JointType_KneeLeft]);
	pointsPants[5] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.leftOuterKnee, 5, GREEN_8U, FILLED, LINE_8);

	csp = DepthSpaceToColorSpace(m_tracePoints.leftInnerKnee.x, m_tracePoints.leftInnerKnee.y);
	offset = GetOffsetForJoint(m_joints[JointType_KneeLeft]);
	pointsPants[6] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.leftInnerKnee, 5, GREEN_8U, FILLED, LINE_8);

	// Right Knee
	Point pointRightHip = m_jointsDepthSpace[JointType_HipRight];
	Point pointRightKnee = m_jointsDepthSpace[JointType_KneeRight];
	slope = -((float)(pointRightHip.x - pointRightKnee.x)) / (pointRightHip.y - pointRightKnee.y);
	m_tracePoints.rightOuterKnee = findBoundary(m_matDepthRaw, pointRightKnee, true, slope);
	m_tracePoints.rightInnerKnee = findBoundary(m_matDepthRaw, pointRightKnee, false, slope);
	
	csp = DepthSpaceToColorSpace(m_tracePoints.rightOuterKnee.x, m_tracePoints.rightOuterKnee.y);
	offset = GetOffsetForJoint(m_joints[JointType_KneeRight]);
	pointsPants[8] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.rightOuterKnee, 5, GREEN_8U, FILLED, LINE_8);

	csp = DepthSpaceToColorSpace(m_tracePoints.rightInnerKnee.x, m_tracePoints.rightInnerKnee.y);
	offset = GetOffsetForJoint(m_joints[JointType_KneeRight]);
	pointsPants[7] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.rightInnerKnee, 5, GREEN_8U, FILLED, LINE_8);


	// Left Quad
	Point quadLeft = pointAverage(pointLeftHip, pointLeftKnee);
	m_tracePoints.leftOuterQuad = findBoundary(m_matDepthRaw, quadLeft, false, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftOuterQuad.x, m_tracePoints.leftOuterQuad.y);
	offset = GetOffsetForJoint(m_joints[JointType_KneeLeft]);
	pointsPants[2] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.leftOuterQuad, 5, GREEN_8U, FILLED, LINE_8);

	// Right Quad
	Point quadRight = pointAverage(pointRightHip, pointRightKnee);
	m_tracePoints.rightOuterQuad = findBoundary(m_matDepthRaw, quadRight, true, 0.0f);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightOuterQuad.x, m_tracePoints.rightOuterQuad.y);
	offset = GetOffsetForJoint(m_joints[JointType_KneeRight]);
	pointsPants[4] = Point(csp.X, csp.Y) + offset;
	circle(m_matColor, Point(csp.X, csp.Y) + offset, 5, GREEN, FILLED, LINE_8);
	circle(m_matDepth, m_tracePoints.rightOuterQuad, 5, GREEN_8U, FILLED, LINE_8);

	namedWindow("Connected Components", WINDOW_NORMAL);
	namedWindow("Connected Components with Landmarks", WINDOW_NORMAL);
	namedWindow("Color with Landmarks", WINDOW_NORMAL);

	imshow("Connected Components", m_matDepthRaw);
	imshow("Connected Components with Landmarks", m_matDepth);
	imshow("Color with Landmarks", m_matColor);

	waitKey(0);

	return pointsPants;
}
