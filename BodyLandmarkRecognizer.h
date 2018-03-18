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

	std::vector<cv::Point> recognizeFor(ClothingType clothingType);
	cv::Point findBoundary(cv::Mat matDepth, cv::Point start, bool traverseRight, float slope);

private:
	static const int cNumTracePointsShirt = 12;
	cv::Point m_tracePointsShirt[12];
	typedef enum _TracePointsShirt {
		TPS_LeftNeck,
		TPS_RightNeck,
		TPS_LeftShoulder,
		TPS_RightShoulder,
		TPS_LeftOuterHem,
		TPS_LeftInnerHem,
		TPS_RightOuterHem,
		TPS_RightInnerHem,
		TPS_LeftHip,
		TPS_RightHip,
		TPS_LeftHipUpper,
		TPS_RightHipUpper
	} TracePointsShirt;

	static const int cNumTracePointsShorts = 9;
	cv::Point m_tracePointsShorts[cNumTracePointsShorts];
	typedef enum _TracePointsShorts {
		TPSH_LeftHip,
		TPSH_RightHip,
		TPSH_Crotch,
		TPSH_LeftOuterKnee,
		TPSH_LeftInnerKnee,
		TPSH_RightOuterKnee,
		TPSH_RightInnerKnee,
		TPSH_LeftOuterQuad,
		TPSH_RightOuterQuad
	} TracePointsShorts;

	UINT16 *m_depthBuffer;
	int m_depthBufferHeight, m_depthBufferWidth;

	RGBQUAD *m_colorBuffer;
	int m_colorBufferHeight, m_colorBufferWidth;

	Joint *m_joints;
	cv::Point m_jointsDepthSpace[JointType_Count];

	ICoordinateMapper *m_coordinateMapper;
	cv::Mat m_matDepth, m_matDepthRaw, m_matColor;

	std::vector<cv::Point> recognizeForShirt();
	std::vector<cv::Point> recognizeForShorts();

	cv::Point GetOffsetForJoint(Joint joint);

	// Utility functions
	cv::Point JointToDepthSpace(JointType jointType);
	cv::Point DepthSpaceToColorSpace(cv::Point point);
};
