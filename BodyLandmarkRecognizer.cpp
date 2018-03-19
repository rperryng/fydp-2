#include "stdafx.h"
#include "BodyLandmarkRecognizer.h"

using namespace cv;
using std::vector;

#define NUM_SHIRT_POINTS 12
const BodyLandmarkRecognizer::TracePoints BodyLandmarkRecognizer::cWhiteList_Shirt[] = {
	TP_LeftNeck,
	TP_RightNeck,
	TP_LeftShoulder,
	TP_RightShoulder,
	TP_LeftOuterHem,
	TP_RightOuterHem,
	TP_LeftInnerHem,
	TP_RightInnerHem,
	TP_LeftHip,
	TP_RightHip,
	TP_LeftUpperHip,
	TP_RightUpperHip
};

#define NUM_SHORT_POINTS 9
const BodyLandmarkRecognizer::TracePoints BodyLandmarkRecognizer::cWhiteList_Shorts[] = {
	TP_LeftUpperHip,
	TP_RightUpperHip,
	TP_LeftOuterQuad,
	TP_Crotch,
	TP_RightOuterQuad,
	TP_LeftOuterKnee,
	TP_LeftInnerKnee,
	TP_RightInnerKnee,
	TP_RightOuterKnee
};

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

	m_depthPoints = vector<Point>(TP_Count);
	m_colorPoints = vector<Point>(TP_Count);

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
	return ((float) (p1.y - p2.y)) / (p1.x - p2.x);
}

Point pointAverage(Point p1, Point p2) {
	return Point(
		(p1.x + p2.x) / 2,
		(p1.y + p2.y) / 2
	);
}

/*
Adds depthPoint to m_depthPoints member variable (vector for depth trace points)
Converts depthPoint to colour basis (with offset) and adds that point to m_colorPoints (vector for colour basis trace points)
*/
void BodyLandmarkRecognizer::convertAndAddPoint(Point depthPoint, JointType jointOffset, TracePoints tracePoint) {
	Point pointColorSpace = DepthSpaceToColorSpace(depthPoint);
	pointColorSpace += GetOffsetForJoint(m_joints[jointOffset]);
	m_depthPoints[tracePoint] = depthPoint;
	m_colorPoints[tracePoint] = pointColorSpace;
}

vector<Point> BodyLandmarkRecognizer::buildTracePoints() {
	vector<Point> pointsDepth(TP_Count);
	vector<Point> pointsColor(TP_Count);
	Point pointColorSpace;
	Point offset;
	Point delta;
	float slope;

	// Neck
	Point pointLeftNeck = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_Neck], false, 0.0f);
	convertAndAddPoint(pointLeftNeck, JointType_Neck, TP_LeftNeck);

	Point pointRightNeck = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_Neck], true, 0.0f);
	convertAndAddPoint(pointRightNeck, JointType_Neck, TP_RightNeck);

	// Shoulder Left
	Point pointLeftShoulder = findBoundary(
		m_matDepthRaw,
		m_jointsDepthSpace[JointType_ShoulderLeft],
		false,
		1.0f
	);
	convertAndAddPoint(pointLeftShoulder, JointType_ShoulderLeft, TP_LeftShoulder);

	// Right Shoulder
	Point pointRightShoulder = findBoundary(
		m_matDepthRaw,
		m_jointsDepthSpace[JointType_ShoulderRight],
		true,
		-1.0f
	);
	convertAndAddPoint(pointRightShoulder, JointType_ShoulderRight, TP_RightShoulder);

	// Left Hip
	Point pointHipLeft = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_HipLeft], false, 0.0f);
	convertAndAddPoint(pointHipLeft, JointType_HipLeft, TP_LeftHip);

	// Right Hip
	Point pointHipRight = findBoundary(m_matDepthRaw, m_jointsDepthSpace[JointType_HipRight], true, 0.0f);
	convertAndAddPoint(pointHipRight, JointType_HipRight, TP_RightHip);

	// Left Hem
	Point pointLeftElbow = m_jointsDepthSpace[JointType_ElbowLeft];
	pointLeftShoulder = m_jointsDepthSpace[JointType_ShoulderLeft];
	Point leftBicep = pointAverage(pointLeftElbow, m_jointsDepthSpace[JointType_ShoulderLeft]);
	slope = -1.0f / calculateSlope(pointLeftElbow, pointLeftShoulder);
	// slope = -((float) (pointLeftElbow.x - pointLeftShoulder.x)) / (pointLeftElbow.y - pointLeftShoulder.y);
	Point pointLeftOuterHem = findBoundary(m_matDepthRaw, leftBicep, false, slope);
	convertAndAddPoint(pointLeftOuterHem, JointType_ElbowLeft, TP_LeftOuterHem);

	delta = leftBicep - pointLeftOuterHem;
	Point pointLeftInnerHem = leftBicep + delta;
	convertAndAddPoint(pointLeftInnerHem, JointType_ElbowLeft, TP_LeftInnerHem);

	// Right Hem
	Point pointRightElbow = m_jointsDepthSpace[JointType_ElbowRight];
	pointRightShoulder = m_jointsDepthSpace[JointType_ShoulderRight];
	Point rightBicep = pointAverage(pointRightElbow, pointRightShoulder);
	slope = -1.0f / calculateSlope(pointRightElbow, pointRightShoulder);
	// slope = -((float) (rightElbow.x - pointRightShoulder.x)) / (rightElbow.y - pointRightShoulder.y);
	Point pointRightOuterHem = findBoundary(m_matDepthRaw, rightBicep, true, slope);
	convertAndAddPoint(pointRightOuterHem, JointType_ElbowRight, TP_RightOuterHem);

	delta = rightBicep - pointRightOuterHem;
	Point pointRightInnerHem = rightBicep + delta;
	convertAndAddPoint(pointRightInnerHem, JointType_ElbowRight, TP_RightInnerHem);

	// Left Hip Upper
	Point pointHipLeftUpper = m_jointsDepthSpace[JointType_HipLeft];
	pointHipLeftUpper.x += 5;
	pointHipLeftUpper.y -= 20;
	pointHipLeftUpper = findBoundary(m_matDepthRaw, pointHipLeftUpper, false, 0.0f);
	convertAndAddPoint(pointHipLeftUpper, JointType_HipLeft, TP_LeftUpperHip);

	// Right Hip Upper
	Point pointHipRightUpper = m_jointsDepthSpace[JointType_HipRight];
	pointHipRightUpper.x -= 5;
	pointHipRightUpper.y -= 20;
	pointHipRightUpper = findBoundary(m_matDepthRaw, pointHipRightUpper, true, 0.0f);
	convertAndAddPoint(pointHipRightUpper, JointType_HipRight, TP_RightUpperHip);

	// Pants
	// poggers
	// in the chat
	// rip xqc

	// Crotch
	Point pointCrotch = m_jointsDepthSpace[JointType_SpineBase];
	for (int y = pointCrotch.y; y < m_depthBufferHeight; y++) {
		if (m_matDepthRaw.at<USHORT>(y, pointCrotch.x) == 0) {
			pointCrotch.y = y;
			break;
		}
	}
	convertAndAddPoint(pointCrotch, JointType_SpineBase, TP_Crotch);

	// Left Knee
	Point pointLeftHip = m_jointsDepthSpace[JointType_HipLeft];
	Point pointLeftKnee = m_jointsDepthSpace[JointType_KneeLeft];
	slope = -1.0f / calculateSlope(pointLeftHip, pointLeftKnee);
	// slope = -((float) (pointLeftHip.x - pointLeftKnee.x)) /  (pointLeftHip.y - pointLeftKnee.y);
	Point pointLeftOuterKnee = findBoundary(m_matDepthRaw, pointLeftKnee, false, slope);
	Point pointLeftInnerKnee = findBoundary(m_matDepthRaw, pointLeftKnee, true, slope);
	convertAndAddPoint(pointLeftOuterKnee, JointType_KneeLeft, TP_LeftOuterKnee);
	convertAndAddPoint(pointLeftInnerKnee, JointType_KneeLeft, TP_LeftInnerKnee);

	// Right Knee
	Point pointRightHip = m_jointsDepthSpace[JointType_HipRight];
	Point pointRightKnee = m_jointsDepthSpace[JointType_KneeRight];
	slope = -1.0f / calculateSlope(pointRightHip, pointRightKnee);
	// slope = -((float)(pointRightHip.x - pointRightKnee.x)) / (pointRightHip.y - pointRightKnee.y);
	Point pointRightOuterKnee = findBoundary(m_matDepthRaw, pointRightKnee, true, slope);
	Point pointRightInnerKnee = findBoundary(m_matDepthRaw, pointRightKnee, false, slope);
	convertAndAddPoint(pointRightOuterKnee, JointType_KneeRight, TP_RightOuterKnee);
	convertAndAddPoint(pointRightInnerKnee, JointType_KneeRight, TP_RightInnerKnee);

	// Left Quad
	Point quadLeft = pointAverage(pointLeftHip, pointLeftKnee);
	Point pointLeftOuterQuad = findBoundary(m_matDepthRaw, quadLeft, false, 0.0f);
	convertAndAddPoint(pointLeftOuterQuad, JointType_HipLeft, TP_LeftOuterQuad);

	// Right Quad
	Point quadRight = pointAverage(pointRightHip, pointRightKnee);
	Point pointRightOuterQuad = findBoundary(m_matDepthRaw, quadRight, true, 0.0f);
	convertAndAddPoint(pointRightOuterQuad, JointType_HipRight, TP_RightOuterQuad);


	for (int i = 0; i < TP_Count; i++) {
		circle(m_matColor, m_colorPoints[i], 5, BLUE, FILLED, LINE_8);
		circle(m_matDepth, m_depthPoints[i], 5, BLUE_8U, FILLED, LINE_8);
	}

	namedWindow("matDepth", WINDOW_NORMAL);
	namedWindow("matColor", WINDOW_NORMAL);
	imshow("matDepth", m_matDepth);
	imshow("matColor", m_matColor);
	waitKey(0);

	return m_depthPoints;
}

vector<Point> BodyLandmarkRecognizer::returnPointsFor(ClothingType clothingType) {
	switch (clothingType)
	{
	case ClothingType_Shirt:
		return filterPoints(cWhiteList_Shirt, NUM_SHIRT_POINTS);
		
	case ClothingType_Shorts:
		return filterPoints(cWhiteList_Shorts, NUM_SHORT_POINTS);
		
	default:
		throw new std::invalid_argument("Invalid clothingType passed to returnPointsFor in BodyLandmarkRecognizer");
	}
}

vector<Point> BodyLandmarkRecognizer::filterPoints(const TracePoints* whitelist, int numElements) {
	vector<Point> filteredPoints(numElements);
	for (int i = 0; i < numElements; i++) {
		filteredPoints[i] = m_colorPoints[whitelist[i]];
	}
	return filteredPoints;
}
