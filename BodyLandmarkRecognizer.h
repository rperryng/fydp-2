#pragma once

typedef enum _ClothingType {
	ClothingType_Shirt,
	ClothingType_Shorts
} ClothingType;

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
	struct tracePoints_t {
		cv::Point leftNeck;
		cv::Point rightNeck;
		cv::Point leftShoulder;
		cv::Point rightShoulder;
		cv::Point leftOuterHem;
		cv::Point leftInnerHem;
		cv::Point rightOuterHem;
		cv::Point rightInnerHem;
		cv::Point leftHip;
		cv::Point rightHip;
		cv::Point leftHipUpper;
		cv::Point rightHipUpper;

		cv::Point crotch;
		cv::Point leftOuterKnee;
		cv::Point leftInnerKnee;
		cv::Point rightOuterKnee;
		cv::Point rightInnerKnee;
		cv::Point leftOuterQuad;
		cv::Point rightOuterQuad;
	};

	UINT16 *m_depthBuffer;
	int m_depthBufferHeight, m_depthBufferWidth;

	RGBQUAD *m_colorBuffer;
	int m_colorBufferHeight, m_colorBufferWidth;

	Joint *m_joints;
	cv::Point m_jointsDepthSpace[JointType_Count];
	tracePoints_t m_tracePoints;

	ICoordinateMapper *m_coordinateMapper;
	cv::Mat m_matDepth, m_matDepthRaw, m_matColor;

	std::vector<cv::Point> recognizeForShirt();
	std::vector<cv::Point> recognizeForShorts();

	cv::Point GetOffsetForJoint(Joint joint);

	// Utility functions
	cv::Point JointToDepthSpace(JointType jointType);
	ColorSpacePoint DepthSpaceToColorSpace(int x, int y);
};
