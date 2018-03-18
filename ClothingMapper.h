#pragma once

class ClothingMapper {
private:
	cv::Mat m_personImage;

	void MapTriangle(
		std::vector<cv::Point> &source_t,
		std::vector<cv::Point> &destination_t,
		std::vector<std::pair<cv::Point, cv::Point>> cutoffLines,
		cv::Mat clothingImage
	);

public:
	ClothingMapper(cv::Mat *personImage);

	void ApplyClothing(
		const int triangles[][3],
		int numTriangles,
		cv::Mat matClothing,
		std::vector<cv::Point> clothingPoints,
		std::vector<cv::Point> bodyPoints,
		bool drawTriangles
	);
};

