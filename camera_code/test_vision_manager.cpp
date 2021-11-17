#include "VisionManager.h"

#include <vector>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

int main()
{
	VisionManager VisManager = VisionManager();

	cv::Mat left_img, right_img;
	std::vector<cv::Point2f> corners_left, corners_right;
	int map[16][2];
	
	/* Chose a value between 0 , 1 and 2. This refers to the image pair the OFFLINE_MODE will use. Once
	 * I fix everything, this will be 0 or 1 depending on which camera we want to use. Till then, you can 
	 * use the OFFLINE_MODE which mimics taking a pair of images from the camera. 
	 */
	int image_pair = 0;

	bool found_pattern = VisManager.get_patten_info_from_device(image_pair, left_img, corners_left, right_img, corners_right, map);
	
	
	printf("\nFound pattern in both: %s\n", found_pattern ? "true" : "false");
	printf("Corners in the left image:\n");
	int count = 0;
	for (cv::Point2f point : corners_left)
	{
		printf("corner %2d at col: %f row: %f \n", count++, point.x, point.y);
	}

	printf("\nCorners in the right image:\n");
	count = 0;
	for (cv::Point2f point : corners_right)
	{
		printf("corner %2d at col: %f row: %f \n", count++, point.x, point.y);
	}

	cv::Mat dst;
	hconcat(left_img, right_img, dst);
	cv::namedWindow("infrared_stereo_pair", cv::WINDOW_NORMAL);
	imshow("infrared_stereo_pair", dst);

	cv::drawChessboardCorners(left_img, cv::Size(4,4), cv::Mat(corners_left), true);
	cv::drawChessboardCorners(right_img, cv::Size(4, 4), cv::Mat(corners_right), true);

	cv::Mat dst_corner;
	hconcat(left_img, right_img, dst_corner);
	cv::namedWindow("infrared_stereo_pair_w_corners", cv::WINDOW_NORMAL);
	imshow("infrared_stereo_pair_w_corners", dst_corner);
	cv::waitKey(0);


	return 1;
}