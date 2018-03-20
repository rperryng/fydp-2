#pragma once

class BodyLandmarkRecognizer {
public:
	BodyLandmarkRecognizer(
		UINT16* depthBuffer,
		int bufferHeight,
		int bufferWidth,
		RGBQUAD *colorBuffer,
		int colorBufferHeight,
		int colorBufferwidth,
		Joint *joints,
		ICoordinateMapper *coordinateMapper
	);

	std::vector<cv::Point> buildTracePoints();
	std::vector<cv::Point> returnPointsFor(ClothingType clothingType);
	cv::Point findBoundary(cv::Mat matDepth, cv::Point start, bool traverseRight, float slope = 0.0f);

private:
	cv::Point m_tracePoints[21];
	typedef enum _TracePoints {
		TP_LeftNeck,
		TP_RightNeck,
		TP_LeftShoulder,
		TP_RightShoulder,
		TP_LeftOuterHem,
		TP_LeftInnerHem,
		TP_RightInnerHem,
		TP_RightOuterHem,
		TP_LeftOuterRib,
		TP_RightOuterRib,
		TP_LeftUpperHip,
		TP_RightUpperHip,
		TP_LeftHip,
		TP_RightHip,
		TP_Crotch,
		TP_LeftOuterKnee,
		TP_LeftInnerKnee,
		TP_RightOuterKnee,
		TP_RightInnerKnee,
		TP_LeftOuterQuad,
		TP_RightOuterQuad,

		TP_Count
	} TracePoints;

	static const TracePoints cWhiteList_Shirt[];
	static const TracePoints cWhiteList_Shorts[];

	static const int cNumTracePointsShirt = 14;

	static const int cNumTracePointsShorts = 9;

	UINT16 *m_depthBuffer;
	int m_depthBufferHeight, m_depthBufferWidth;

	RGBQUAD *m_colorBuffer;
	int m_colorBufferHeight, m_colorBufferWidth;

	Joint *m_joints;
	cv::Point m_jointsDepthSpace[JointType_Count];

	ICoordinateMapper *m_coordinateMapper;
	cv::Mat m_matDepth, m_matDepthRaw, m_matColor;

	std::vector<cv::Point> m_depthPoints;
	std::vector<cv::Point> m_colorPoints;

	std::vector<cv::Point> filterPoints(const TracePoints* whitelist, int numElements);
	cv::Point GetOffsetForJoint(Joint joint);

	// Utility functions
	cv::Point JointToDepthSpace(JointType jointType);
	cv::Point DepthSpaceToColorSpace(cv::Point point);

	void convertAndAddPoint(cv::Point depthPoint, JointType jointForOffset, TracePoints tracePoint);
};
