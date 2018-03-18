#include "stdafx.h"
#include "ClothingMapper.h"

using std::vector;
using std::pair;
using namespace cv;

int directionOfPoint(pair<Point, Point> line, Point c) {
	Point a = line.first;
	Point b = line.second;
	return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x));
}

vector<Point> cornersOfRect(Rect rectangle) {
	vector<Point> corners;
	Point origin(rectangle.x, rectangle.y);
	corners.push_back(origin);
	corners.push_back(origin + Point(rectangle.width - 1, 0));
	corners.push_back(origin + Point(0, rectangle.height - 1));
	corners.push_back(origin + Point(rectangle.width - 1, rectangle.height - 1));
	return corners;
}

bool sameSign(int x, int y) {
	return (y >= 0) ^ (x < 0);
}

bool vectorContains(vector<Point> container, Point value) {
	return find(container.begin(), container.end(), value) != container.end();
}

ClothingMapper::ClothingMapper(Mat *personImage) {
	m_personImage = *personImage;
}

void ClothingMapper::MapTriangle(
	vector<Point> &source_t,
	vector<Point> &destination_t,
	vector<pair<Point, Point>> cutoffLines,
	Mat clothingImage
) {
	// Find bounding rectangle for each triangle
	Rect source_rect = boundingRect(source_t);
	Rect destination_rect = boundingRect(destination_t);

	// Offset pointsShirt by left top corner of the respective rectangles
	Point2f source_triangle_bounded[3];
	Point2f destination_triangle_bounded[3];
	Point destination_triangle_bounded_int[3];

	for (int i = 0; i < 3; i++) {
		source_triangle_bounded[i] = Point2f(source_t[i].x - source_rect.x, source_t[i].y - source_rect.y);
		destination_triangle_bounded[i] = Point2f(destination_t[i].x - destination_rect.x, destination_t[i].y - destination_rect.y);
		destination_triangle_bounded_int[i] = Point(destination_t[i].x - destination_rect.x, destination_t[i].y - destination_rect.y);
	}

	//	Get mask by filling triangle
	vector<Mat> masks;
	for (int i = 0; i < cutoffLines.size(); i++) {
		pair<Point, Point> cutoffLine = cutoffLines[i];
		Point thirdPoint;

		for (int j = 0; j < destination_t.size(); j++) {
			if (destination_t[j] == cutoffLine.first) continue;
			if (destination_t[j] == cutoffLine.second) continue;
			thirdPoint = destination_t[j];
		}

		int thirdPointDirection = directionOfPoint(cutoffLine, thirdPoint);
		Point maskCorner;
		bool foundCorner = false;
		vector<Point> corners = cornersOfRect(destination_rect);

		for (int j = 0; j < corners.size(); j++) {
			Point corner = corners[j];
			//circle(m_personImage, corner, 5, GREEN_8U, FILLED, LINE_8);

			int cornerDirection = directionOfPoint(cutoffLine, corner);
			if (cornerDirection != 0 && !sameSign(thirdPointDirection, cornerDirection)) {
				foundCorner = true;
				maskCorner = corner;
			}
		}

		if (foundCorner) {
			Mat mask = Mat::zeros(destination_rect.height, destination_rect.width, CV_32FC4);
			Point origin(destination_rect.x, destination_rect.y);
			Point maskTriangle[3] = {
				cutoffLine.first - origin,
				cutoffLine.second - origin,
				maskCorner - origin
			};

			cv::fillConvexPoly(mask, maskTriangle, 3, Scalar(1.0, 1.0, 1.0, 1.0));
			mask = Scalar(1.0, 1.0, 1.0, 1.0) - mask;
			cv::line(mask, cutoffLine.first - origin, cutoffLine.second - origin, Scalar(1.0, 1.0, 1.0, 1.0), 1);
			masks.push_back(mask);
		}
	}

	//	Apply warpImage to small rectangular patches
	Mat source_crop;
	clothingImage(source_rect).copyTo(source_crop);

	Mat dest_crop = Mat::zeros(destination_rect.height, destination_rect.width, source_crop.type());

	// Given a pair of triangles, find the affine transform.
	Mat warp_mat = getAffineTransform(source_triangle_bounded, destination_triangle_bounded);

	// Apply the Affine Transform just found to the src image
	cv::warpAffine(source_crop, dest_crop, warp_mat, dest_crop.size());

	Mat alpha_mask;
	cv::extractChannel(dest_crop, alpha_mask, 3);
	cvtColor(alpha_mask, alpha_mask, COLOR_GRAY2BGRA);

	for (int i = 0; i < masks.size(); i++) {
		cv::multiply(alpha_mask, masks[i], alpha_mask);
	}

	cv::multiply(dest_crop, alpha_mask, dest_crop);
	cv::multiply(m_personImage(destination_rect), Scalar(1.0, 1.0, 1.0, 1.0) - alpha_mask, m_personImage(destination_rect));
	m_personImage(destination_rect) = m_personImage(destination_rect) + dest_crop;
}

void ClothingMapper::ApplyClothing(
	const int triangles[][3],
	int numTriangles,
	Mat matClothing,
	vector<Point> clothingPoints,
	vector<Point> bodyPoints,
	bool drawTriangles
) {
	vector<vector<Point>> sourceTriangles;
	vector<vector<Point>> destinationTriangles;
	for (int i = 0; i < numTriangles; i++) {
		vector<Point> source_t, dest_t;
		for (int j = 0; j < 3; j++) {
			source_t.push_back(clothingPoints[triangles[i][j]]);
			dest_t.push_back(bodyPoints[triangles[i][j]]);
		}

		sourceTriangles.push_back(source_t);
		destinationTriangles.push_back(dest_t);
	}

	vector<pair<Point, Point>> allCutoffLines;
	for (int i = 0; i < numTriangles; i++) {
		vector<Point> source_t = sourceTriangles[i];
		vector<Point> destination_t = destinationTriangles[i];
		vector<pair<Point, Point>> cutoffLines;

		for (int j = 0; j < numTriangles; j++) {
			if (i == j) continue;
			vector<Point> other_t = destinationTriangles[j];

			for (int k = 0; k < destination_t.size(); k++) {
				Point start = destination_t[k];
				Point end = destination_t[(k == destination_t.size() - 1) ? 0 : k + 1];
				if (vectorContains(other_t, start) && vectorContains(other_t, end)) {
					cutoffLines.push_back(pair<Point, Point>(start, end));
					allCutoffLines.push_back(pair<Point, Point>(start, end));
				}
			}
		}

		MapTriangle(source_t, destination_t, cutoffLines, matClothing);
	}

	// Draw Triangles
	if (!drawTriangles) return;

	for (int i = 0; i < destinationTriangles.size(); i++) {
		vector<Point> currentTriangle = destinationTriangles[i];
		for (int j = 0; j < currentTriangle.size(); j++) {
			Point start = currentTriangle[j];
			Point end = (j == currentTriangle.size() - 1) ? currentTriangle[0] : currentTriangle[j + 1];
			line(m_personImage, start, end, RED_8U, 2);
		}
	}

	// Draw cutoff lines
	for (int j = 0; j < allCutoffLines.size(); j++) {
		pair<Point, Point> cutoffLine = allCutoffLines[j];
		line(m_personImage, cutoffLine.first, cutoffLine.second, GREEN_8U, 2);
	}

}

