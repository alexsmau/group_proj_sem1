#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <stdio.h>
#include <cmath>
#include <librealsense2/rs.hpp>
#include <vector>
#include "VisionManager.h"
#include <opencv2/core/types_c.h>


using namespace cv;
using namespace std;

vector <float> disparity;
vector <float> depth;
vector <float> azimuth;
vector <float> inclination;
vector <vector<float>> patterncenter;


void depth_calc(float k1_x, float k2_x, int i) {

	float focal_length = 1.93;
	int baseline = 50;

	//float x_FOV_dist = 848 / 91.2;

	float x_disp = (k1_x - k2_x);
	disparity.push_back(x_disp);

	cout << "disparity " << i << " is = " << disparity[i] << endl;
	// cout << "disparity vector = " << disparity[0]  << endl;

	float x_depth = (focal_length * baseline) / disparity[i];
	depth.push_back(x_depth);

	cout << "depth for point " << i << " is = " << depth[i] << endl;
}
// 848x 480y
//    depth imager // 74 horizontal FOV;  62 vertical FOV;  88 diagonal FOV
// standard imager // 69 horizontal FOV; 42.5 vertical FOV; 77 diagonal FOV
// WIDE imager   // 91.2 horizontal FOV; 65.5 vertical FOV; 100.6 diagonal FOV

void angle_calc(int x, int y, int i) {

	float x_angle = (x - 424) * (91.2 / 848);
	float y_angle = -1 * (y - 240) * (65.5 / 480);
	azimuth.push_back(x_angle);
	inclination.push_back(y_angle);

	//cout << "x angle for point " << i << " is = " << x_angle << endl;
	//cout << "y angle for point " << i << " is = " << y_angle << endl;
	cout << " " << endl;
}

void find_average_vector() {
	//add all vector things together
	float sum_azimuth = 0;
	float sum_inclination = 0;
	float sum_depth = 0;

	for (int i = 0; i < 16; i++)
	{
		
		sum_azimuth = sum_azimuth + azimuth[i];
		sum_inclination = sum_inclination + inclination[i];
		sum_depth = sum_depth + depth[i];

	}

	float average_azimuth = sum_azimuth / 16;
	float average_inclination = sum_inclination / 16;
	float average_depth = sum_depth / 16;

	//convert from polar to cartesian
	float x = average_depth * cos(average_azimuth) * sin(average_inclination);
	float y = average_depth * sin(average_azimuth) * sin(average_inclination);
	float z = average_depth * cos(average_inclination);

	cout << "x is = " << x << " y is = " << y << " z is = " << z << endl;
	vector <float> xyz = { x, y, z};
	patterncenter.push_back(xyz);
}


int main() {

	VisionManager VisManager = VisionManager();

	cv::Mat left_img, right_img;
	std::vector<cv::Point2f> corners_left, corners_right;
	int map[16][2];
	
	/* Chose a value between 0 , 1 and 2. This refers to the image pair the OFFLINE_MODE will use. Once
	 * I fix everything, this will be 0 or 1 depending on which camera we want to use. Till then, you can
	 * use the OFFLINE_MODE which mimics taking a pair of images from the camera.
	 */
	int image_pair = 1;

	bool found_pattern = VisManager.get_patten_info_from_device(image_pair, left_img, corners_left, right_img, corners_right, map);

	for (int i = 0; i < 16; i++)
	{
		float x_l = corners_left[map[i][0]].x;
		float x_r = corners_right[map[i][1]].x;

		//cout << "corner x positions = " << x_l << " and " << x_r << endl;
		depth_calc(x_l, x_r, i);
		angle_calc(corners_left[map[i][0]].x, corners_left[map[i][0]].y, i);
		
	}
	find_average_vector();
	// x, y and z of the center of the pattern is cout'ed in the function. 
	//They are also stored in "patterncenter" as vector<vector<float>> but are not used in that form.


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

	cv::drawChessboardCorners(left_img, cv::Size(4, 4), cv::Mat(corners_left), true);
	cv::drawChessboardCorners(right_img, cv::Size(4, 4), cv::Mat(corners_right), true);

	cv::Mat dst_corner;
	hconcat(left_img, right_img, dst_corner);
	cv::namedWindow("infrared_stereo_pair_w_corners", cv::WINDOW_NORMAL);
	imshow("infrared_stereo_pair_w_corners", dst_corner);
	cv::waitKey(0);


	return 1;
}



