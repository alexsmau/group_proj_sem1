#include "VisionManager.h"

#include <vector>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <librealsense2/rs.hpp>

#define GLOBAL_CAM_INDEX (0)
#define LOCAL_CAM_INDEX (1)

std::vector <float> disparity;
std::vector <float> depth;
std::vector <float> azimuth;
std::vector <float> inclination;

void depth_calc(float k1_x, float k2_x, int i) {

	float focal_length = 1.93;
	int baseline = 50;

	//float x_FOV_dist = 848 / 91.2;

	float x_disp = (k1_x - k2_x);
	disparity.push_back(x_disp);

	std::cout << "disparity " << i << " is = " << disparity[i] << std::endl;
	// cout << "disparity vector = " << disparity[0]  << endl;

	float x_depth = (focal_length * baseline) / disparity[i];
	depth.push_back(x_depth);

	std::cout << "depth for point " << i << " is = " << depth[i] << std::endl;
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
	std::cout << " " << std::endl;
}

void get_3d_coordinated(std::vector<cv::Point3f> &points_3d)
{
	points_3d.clear();
	for (int i = 0; i < 16; i++)
	{
		float x = depth[i] * cos(azimuth[i]) * sin(inclination[i]);
		float y = depth[i] * sin(azimuth[i]) * sin(inclination[i]);
		float z = depth[i] * cos(inclination[i]);
		
		std::cout << "x = " << x << " y = " << y << " z = " << z << "\n";

		points_3d.push_back(cv::Point3f(x, y, z));
	}
}

int main()
{
	VisionManager VisManager = VisionManager();
	int count = 0;

	std::cout << "Iteration: " << ++count << "\n";
	cv::Mat left_img, right_img;
	std::vector<cv::Point2f> corners_left, corners_right;
	int map[16][2];
	bool found_pattern = VisManager.get_patten_info_from_device(LOCAL_CAM_INDEX, left_img, corners_left, right_img, corners_right, map);

	printf("\nLocal found pattern in both: %s\n", found_pattern ? "true" : "false");

	cv::Mat dst;
	hconcat(left_img, right_img, dst);
	cv::imwrite("local_left_img5.png", left_img);
	cv::imwrite("local_right_img5.png", right_img);
	//cv::namedWindow("infrared_stereo_pair", cv::WINDOW_NORMAL);
	//imshow("infrared_stereo_pair", dst);
	if (found_pattern)
	{
		cv::drawChessboardCorners(left_img, cv::Size(4, 4), cv::Mat(corners_left), true);
		cv::drawChessboardCorners(right_img, cv::Size(4, 4), cv::Mat(corners_right), true);


		cv::Mat local_dst_corner;
		hconcat(left_img, right_img, local_dst_corner);
		cv::namedWindow("local_infrared_stereo_pair_w_corners", cv::WINDOW_NORMAL);
		imshow("local_infrared_stereo_pair_w_corners", local_dst_corner);

		std::vector<cv::Point3f> objectPoints;
		for (int i = 0; i < 16; i++)
		{
			float x_l = corners_left[map[i][0]].x;
			float x_r = corners_right[map[i][1]].x;

			//cout << "corner x positions = " << x_l << " and " << x_r << endl;
			depth_calc(x_l, x_r, i);
			angle_calc(corners_left[map[i][0]].x, corners_left[map[i][0]].y, i);
		}
		get_3d_coordinated(objectPoints);

		for (int i = 0; i < 16; i++)
		{
			std::cout << "x = " << objectPoints[i].x << " y = " << objectPoints[i].y << " z = " << objectPoints[i].z << "\n";
		}
	}


	cv::waitKey(0);

	return 1;
}