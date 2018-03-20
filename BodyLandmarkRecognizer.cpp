#include "stdafx.h"
#include "BodyLandmarkRecognizer.h"

using namespace cv;
using std::vector;

#define NUM_SHIRT_POINTS 14
const BodyLandmarkRecognizer::TracePoints BodyLandmarkRecognizer::cWhiteList_Shirt[] = {
	TP_LeftNeck,
	TP_RightNeck,
	TP_LeftShoulder,
	TP_RightShoulder,
	TP_LeftOuterHem,
	TP_RightOuterHem,
	TP_LeftInnerHem,
	TP_RightInnerHem,
	TP_LeftOuterRib,
	TP_RightOuterRib,
	TP_LeftUpperHip,
	TP_RightUpperHip,
	TP_LeftHip,
	TP_RightHip
};

#define NUM_SWEATER_POINTS 22
const BodyLandmarkRecognizer::TracePoints BodyLandmarkRecognizer::cWhiteList_Sweater[] = {
	TP_LeftNeck,
	TP_RightNeck,
	TP_LeftShoulder,
	TP_RightShoulder,
	TP_LeftOuterHem,
	TP_RightOuterHem,
	TP_LeftInnerHem,
	TP_RightInnerHem,
	TP_LeftOuterRib,
	TP_RightOuterRib,
	TP_LeftUpperHip,
	TP_RightUpperHip,
	TP_LeftHip,
	TP_RightHip,
	TP_LeftOuterElbow,
	TP_RightOuterElbow,
	TP_LeftInnerElbow,
	TP_RightInnerElbow,
	TP_LeftOuterWrist,
	TP_RightOuterWrist,
	TP_LeftInnerWrist,
	TP_RightInnerWrist
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

Point BodyLandmarkRecognizer::findBoundary(Point start, bool traverseRight, float slope) {
	if (traverseRight) {
		for (int x = start.x; x < m_depthBufferWidth; x++) {
			int y = start.y + (slope * (x - start.x));

			if (m_matDepthRaw.at<USHORT>(y, x) == 0) {
				return Point(x, y);
			}
		}
	} else {
		for (int x = start.x; x >= 0; x--) {
			int y = start.y + (slope * (x - start.x));

			if (m_matDepthRaw.at<USHORT>(y, x) == 0) {
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
	Point pointLeftNeck = findBoundary(m_jointsDepthSpace[JointType_Neck], false, 0.0f);
	convertAndAddPoint(pointLeftNeck, JointType_Neck, TP_LeftNeck);

	Point pointRightNeck = findBoundary(m_jointsDepthSpace[JointType_Neck], true, 0.0f);
	convertAndAddPoint(pointRightNeck, JointType_Neck, TP_RightNeck);

	// Shoulder Left
	Point pointLeftShoulder = findBoundary(
		m_jointsDepthSpace[JointType_ShoulderLeft],
		false,
		1.0f
	);
	convertAndAddPoint(pointLeftShoulder, JointType_ShoulderLeft, TP_LeftShoulder);

	// Right Shoulder
	Point pointRightShoulder = findBoundary(
		m_jointsDepthSpace[JointType_ShoulderRight],
		true,
		-1.0f
	);
	convertAndAddPoint(pointRightShoulder, JointType_ShoulderRight, TP_RightShoulder);

	// Left Hip
	Point pointHipLeft = findBoundary(m_jointsDepthSpace[JointType_HipLeft], false, 0.0f);
	convertAndAddPoint(pointHipLeft, JointType_HipLeft, TP_LeftHip);

	// Right Hip
	Point pointHipRight = findBoundary(m_jointsDepthSpace[JointType_HipRight], true, 0.0f);
	convertAndAddPoint(pointHipRight, JointType_HipRight, TP_RightHip);

	// Left Hem
	Point leftBicep = pointAverage(m_jointsDepthSpace[JointType_ElbowLeft], m_jointsDepthSpace[JointType_ShoulderLeft]);
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_ElbowLeft], m_jointsDepthSpace[JointType_ShoulderLeft]);
	Point pointLeftOuterHem = findBoundary(leftBicep, false, slope);
	delta = leftBicep - pointLeftOuterHem;
	Point pointLeftInnerHem = leftBicep + delta;
	convertAndAddPoint(pointLeftOuterHem, JointType_ElbowLeft, TP_LeftOuterHem);
	convertAndAddPoint(pointLeftInnerHem, JointType_ElbowLeft, TP_LeftInnerHem);

	// Right Hem
	Point rightBicep = pointAverage(m_jointsDepthSpace[JointType_ElbowRight], m_jointsDepthSpace[JointType_ShoulderRight]);
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_ElbowRight], m_jointsDepthSpace[JointType_ShoulderRight]);
	Point pointRightOuterHem = findBoundary(rightBicep, true, slope);
	delta = rightBicep - pointRightOuterHem;
	Point pointRightInnerHem = rightBicep + delta;
	convertAndAddPoint(pointRightOuterHem, JointType_ElbowRight, TP_RightOuterHem);
	convertAndAddPoint(pointRightInnerHem, JointType_ElbowRight, TP_RightInnerHem);

	// Left Hip Upper
	Point pointHipLeftUpper = m_jointsDepthSpace[JointType_HipLeft];
	pointHipLeftUpper.x += 5;
	pointHipLeftUpper.y -= 20;
	pointHipLeftUpper = findBoundary(pointHipLeftUpper, false, 0.0f);
	convertAndAddPoint(pointHipLeftUpper, JointType_HipLeft, TP_LeftUpperHip);

	// Right Hip Upper
	Point pointHipRightUpper = m_jointsDepthSpace[JointType_HipRight];
	pointHipRightUpper.x -= 5;
	pointHipRightUpper.y -= 20;
	pointHipRightUpper = findBoundary(pointHipRightUpper, true, 0.0f);
	convertAndAddPoint(pointHipRightUpper, JointType_HipRight, TP_RightUpperHip);

	// Left Outer Rib
	Point bellyButtonPoint = pointAverage(m_jointsDepthSpace[JointType_SpineMid], m_jointsDepthSpace[JointType_SpineBase]);
	Point leftOuterRibTP = findBoundary(bellyButtonPoint, false, 0.0f);
	convertAndAddPoint(leftOuterRibTP, JointType_SpineMid, TP_LeftOuterRib);

	// Right Outer Rib
	Point rightOuterRibTp = findBoundary(bellyButtonPoint, true, 0.0f);
	convertAndAddPoint(rightOuterRibTp, JointType_SpineMid, TP_RightOuterRib);

	// Left Elbow
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_ElbowLeft], m_jointsDepthSpace[JointType_ShoulderLeft]);
	Point leftOuterElbowTP = findBoundary(m_jointsDepthSpace[JointType_ElbowLeft], false, slope);
	Point leftInnerElbowTP = findBoundary(m_jointsDepthSpace[JointType_ElbowLeft], true, slope);
	convertAndAddPoint(leftOuterElbowTP, JointType_ElbowLeft, TP_LeftOuterElbow);
	convertAndAddPoint(leftInnerElbowTP, JointType_ElbowLeft, TP_LeftInnerElbow);

	// Right Elbow
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_ElbowRight], m_jointsDepthSpace[JointType_ShoulderRight]);
	Point rightOuterElbowTP = findBoundary(m_jointsDepthSpace[JointType_ElbowRight], true, slope);
	Point rightInnerElbowTP = findBoundary(m_jointsDepthSpace[JointType_ElbowRight], false, slope);
	convertAndAddPoint(rightOuterElbowTP, JointType_ElbowRight, TP_RightOuterElbow);
	convertAndAddPoint(rightInnerElbowTP, JointType_ElbowRight, TP_RightInnerElbow);

	// Left Wrist
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_ElbowLeft], m_jointsDepthSpace[JointType_WristLeft]);
	Point leftOuterWristTP = findBoundary(m_jointsDepthSpace[JointType_WristLeft], false, slope);
	Point leftInnerWristTP = findBoundary(m_jointsDepthSpace[JointType_WristLeft], true, slope);
	convertAndAddPoint(leftOuterWristTP, JointType_WristLeft, TP_LeftOuterWrist);
	convertAndAddPoint(leftInnerWristTP, JointType_WristLeft, TP_LeftInnerWrist);

	// Right Wrist
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_ElbowRight], m_jointsDepthSpace[JointType_WristRight]);
	Point rightOuterWristTP = findBoundary(m_jointsDepthSpace[JointType_WristRight], true, slope);
	Point rightInnerWristTP = findBoundary(m_jointsDepthSpace[JointType_WristRight], false, slope);
	convertAndAddPoint(rightOuterWristTP, JointType_WristRight, TP_RightOuterWrist);
	convertAndAddPoint(rightInnerWristTP, JointType_WristRight, TP_RightInnerWrist);

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
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_HipLeft], m_jointsDepthSpace[JointType_KneeLeft]);
	// slope = -((float) (pointLeftHip.x - pointLeftKnee.x)) /  (pointLeftHip.y - pointLeftKnee.y);
	Point pointLeftOuterKnee = findBoundary(m_jointsDepthSpace[JointType_KneeLeft], false, slope);
	Point pointLeftInnerKnee = findBoundary(m_jointsDepthSpace[JointType_KneeLeft], true, slope);
	convertAndAddPoint(pointLeftOuterKnee, JointType_KneeLeft, TP_LeftOuterKnee);
	convertAndAddPoint(pointLeftInnerKnee, JointType_KneeLeft, TP_LeftInnerKnee);

	// Right Knee
	slope = -1.0f / calculateSlope(m_jointsDepthSpace[JointType_HipRight], m_jointsDepthSpace[JointType_KneeRight]);
	// slope = -((float)(pointRightHip.x - pointRightKnee.x)) / (pointRightHip.y - pointRightKnee.y);
	Point pointRightOuterKnee = findBoundary(m_jointsDepthSpace[JointType_KneeRight], true, slope);
	Point pointRightInnerKnee = findBoundary(m_jointsDepthSpace[JointType_KneeRight], false, slope);
	convertAndAddPoint(pointRightOuterKnee, JointType_KneeRight, TP_RightOuterKnee);
	convertAndAddPoint(pointRightInnerKnee, JointType_KneeRight, TP_RightInnerKnee);

	// Left Quad
	Point quadLeft = pointAverage(m_jointsDepthSpace[JointType_HipLeft], m_jointsDepthSpace[JointType_KneeLeft]);
	Point pointLeftOuterQuad = findBoundary(quadLeft, false, 0.0f);
	convertAndAddPoint(pointLeftOuterQuad, JointType_HipLeft, TP_LeftOuterQuad);

	// Right Quad
	Point quadRight = pointAverage(m_jointsDepthSpace[JointType_HipRight], m_jointsDepthSpace[JointType_KneeRight]);
	Point pointRightOuterQuad = findBoundary(quadRight, true, 0.0f);
	convertAndAddPoint(pointRightOuterQuad, JointType_HipRight, TP_RightOuterQuad);

	// Footz
	circle(m_matDepth, m_jointsDepthSpace[JointType_AnkleLeft], 5, GREEN_16U, FILLED, LINE_8);
	circle(m_matDepth, m_jointsDepthSpace[JointType_AnkleRight], 5, GREEN_16U, FILLED, LINE_8);
	circle(m_matDepth, m_jointsDepthSpace[JointType_FootLeft], 5, RED_16U, FILLED, LINE_8);
	circle(m_matDepth, m_jointsDepthSpace[JointType_FootRight], 5, RED_16U, FILLED, LINE_8);

	for (int i = 0; i < TP_Count; i++) {
		circle(m_matColor, m_colorPoints[i], 5, BLUE_8U, FILLED, LINE_8);
		circle(m_matDepth, m_depthPoints[i], 5, BLUE_16U, FILLED, LINE_8);
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

	case ClothingType_Sweater:
		return filterPoints(cWhiteList_Sweater, NUM_SWEATER_POINTS);

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
