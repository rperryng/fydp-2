#pragma once

class ClothingMapper {
public:
	ClothingMapper(cv::Mat *personImage);

	void ApplyClothing(
		ClothingType clothingType,
		cv::Mat matClothing,
		std::vector<cv::Point> clothingPoints,
		std::vector<cv::Point> bodyPoints,
		bool drawTriangles
	);

private:
	static const int cNumTrianglesShirt = 12;
	static const int cNumTrianglesShorts = 7;
	static const int cNumTrianglesSweater = 20;
	cv::Mat m_personImage;

	void MapTriangle(
		std::vector<cv::Point> &source_t,
		std::vector<cv::Point> &destination_t,
		std::vector<std::pair<cv::Point, cv::Point>> cutoffLines,
		cv::Mat clothingImage
	);

	int getNumTrianglesForClothingType(ClothingType clothingType);
	int** getTrianglesForClothingType(ClothingType clothingType);
};
