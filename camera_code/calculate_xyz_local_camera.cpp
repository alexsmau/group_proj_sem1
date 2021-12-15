#include "VisionManager.h"

#include<Eigen/Dense>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <librealsense2/rs.hpp>

#include <iostream>
#include <vector>

#define GLOBAL_CAM_INDEX (0)
#define LOCAL_CAM_INDEX (1)
#define NR_OF_ITERATIONS (10)

void make_camera_matrix(bool is_local_cam, float cam_mat[3][3])
{
	if (is_local_cam)
	{
		std::cout << "\n Use local camera intrinsics \n";
		cam_mat[0][0] = 646.908;// fx
		cam_mat[0][1] = 0;
		cam_mat[0][2] = 641.877; // cx

		cam_mat[1][0] = 0;
		cam_mat[1][1] = 646.908; //fy
		cam_mat[1][2] = 405.522; //cy

		cam_mat[2][0] = 0;
		cam_mat[2][1] = 0;
		cam_mat[2][2] = 1;
	}
	else
	{
		std::cout << "\n Use global camera intrinsics \n";
		cam_mat[0][0] = 634.637;// fx
		cam_mat[0][1] = 0;
		cam_mat[0][2] = 636.179; // cx

		cam_mat[1][0] = 0;
		cam_mat[1][1] = 634.637; //fy
		cam_mat[1][2] = 393.432; //cy

		cam_mat[2][0] = 0;
		cam_mat[2][1] = 0;
		cam_mat[2][2] = 1;
	}
}

void get_3d_coord_world_frame(std::vector<cv::Point3f>& points_3d)
{
	points_3d.clear();
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			points_3d.push_back(cv::Point3f(i * 29, -j * 29, 0));
		}
	}
}

int main()
{
	/*******************************************************************
	 * STEP 1: Initialize translation matrices for left/right cameras  *
	 *******************************************************************/
	double base_leftGlobalCam_double[4][4] = { { 1, 0, 0, 2.2 },
											   {0, 1, 0, -12.5},
											   {0, 0, 1, 196.1},
											   {0, 0, 0, 1} };
	cv::Mat base_leftGlobalCam_mat(4, 4, CV_64F, &base_leftGlobalCam_double);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> base_leftGlobalCam;
	cv::cv2eigen(base_leftGlobalCam_mat, base_leftGlobalCam);

	double leftLocalCam_endeff_double[4][4] = {{ 1, 0, 0, 18 },
										       { 0, 1, 0, 25 },
								       		   { 0, 0, 1, -43.8 },
										       { 0, 0, 0, 1 } };
	cv::Mat leftLocalCam_endeff_mat(4, 4, CV_64F, &leftLocalCam_endeff_double);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> leftLocalCam_endeff;
	cv::cv2eigen(leftLocalCam_endeff_mat, leftLocalCam_endeff);

	double base_rightGlobalCam_double[4][4] = { {1, 0, 0, 52.2},
	                                     {0, 1, 0, -12.5},
	                                     {0, 0, 1, 196.1},
	                                     {0, 0, 0, 1} };
	cv::Mat base_rightGlobalCam_mat(4, 4, CV_64F, &base_rightGlobalCam_double);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> base_rightGlobalCam;
	cv::cv2eigen(base_rightGlobalCam_mat, base_rightGlobalCam);

	double rightLocalCam_endeff_double[4][4] = { {1, 0, 0, -32},
	                                             {0, 1, 0, 25},
	                                             {0, 0, 1, -43.8},
	                                             {0, 0, 0, 1} };
	cv::Mat rightLocalCam_endeff_mat(4, 4, CV_64F, &rightLocalCam_endeff_double);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> rightLocalCam_endeff;
	cv::cv2eigen(rightLocalCam_endeff_mat, rightLocalCam_endeff);

	/****************************************************************
     * STEP 2: Get information from Cameras                         *
     ****************************************************************/
	int camera_index;

	double position_from_left[NR_OF_ITERATIONS][3];
	double position_from_right[NR_OF_ITERATIONS][3];

	float cam_mat[3][3];
	float coeff[5] = { 0, 0, 0, 0, 0 };
	cv::Mat distortion_coeff(1, 5, CV_32F, &coeff);

	VisionManager VisManager = VisionManager();
	bool found_pattern;
	int map[40][2];
	cv::Mat left_img, right_img;
	std::vector<cv::Point2f> corners_left, corners_right;

	std::vector<cv::Point3f> objectPoints2;

	for (int i = 0; i < NR_OF_ITERATIONS; i++)
	{
		std::cout << "\n ITERATION: " << i + 1 << "\n";
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tmat_global_left;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tmat_global_right;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tmat_local_left;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tmat_local_right;
		/****************************************************************
		 * STEP 2.1: Get information from Global Camera                 *
		 ****************************************************************/
		camera_index = GLOBAL_CAM_INDEX;		
		
		found_pattern = false;
		while (!found_pattern)
		{
			found_pattern = VisManager.get_patten_info_from_device(camera_index, left_img, corners_left, right_img, corners_right, map);

			if (!found_pattern)
			{
				std::cout << "Could not find pattern in global camera.\n";
			}
			else
			{
				make_camera_matrix((camera_index == LOCAL_CAM_INDEX), cam_mat);
				cv::Mat camera_matrix(3, 3, CV_32F, &cam_mat);

				get_3d_coord_world_frame(objectPoints2);

				cv::Mat rvec1, tvec1, outjac, rvec2, tvec2, rmat1, rmat2, jacobian1, jacobian2, conc1, conc2, tmat1, tmat2;
				std::vector<cv::Point2f> outimg;

				cv::solvePnP(objectPoints2, corners_left, camera_matrix, distortion_coeff, rvec1, tvec1);
				cv::solvePnP(objectPoints2, corners_right, camera_matrix, distortion_coeff, rvec2, tvec2);

				cv::projectPoints(objectPoints2, rvec1, tvec1, camera_matrix, distortion_coeff, outimg, outjac);

				cv::Rodrigues(rvec1, rmat1, jacobian1);
				cv::Rodrigues(rvec2, rmat2, jacobian2);

				double bott2[1][4] = { {0.0, 0.0, 0.0, 1.0} };
				cv::Mat bott(1, 4, rmat1.type(), &bott2);

				hconcat(rmat1, tvec1, conc1);
				hconcat(rmat2, tvec2, conc2);

				vconcat(conc1, bott, tmat1);
				vconcat(conc2, bott, tmat2);

				cv::cv2eigen(tmat1, tmat_global_left);
				cv::cv2eigen(tmat2, tmat_global_right);
			}
		}
		
		/****************************************************************
		 * STEP 2.2: Get information from Local Camera                  *
		 ****************************************************************/
		camera_index = LOCAL_CAM_INDEX;

		found_pattern = false;
		while (!found_pattern)
		{
			found_pattern = VisManager.get_patten_info_from_device(camera_index, left_img, corners_left, right_img, corners_right, map);

			if (!found_pattern)
			{
				std::cout << "Could not find pattern in local camera.\n";
			}
			else
			{
				make_camera_matrix((camera_index == LOCAL_CAM_INDEX), cam_mat);
				cv::Mat camera_matrix(3, 3, CV_32F, &cam_mat);

				get_3d_coord_world_frame(objectPoints2);

				cv::Mat rvec1, tvec1, outjac, rvec2, tvec2, rmat1, rmat2, jacobian1, jacobian2, conc1, conc2, tmat1, tmat2;
				std::vector<cv::Point2f> outimg;

				cv::solvePnP(objectPoints2, corners_left, camera_matrix, distortion_coeff, rvec1, tvec1);
				cv::solvePnP(objectPoints2, corners_right, camera_matrix, distortion_coeff, rvec2, tvec2);

				cv::projectPoints(objectPoints2, rvec1, tvec1, camera_matrix, distortion_coeff, outimg, outjac);

				cv::Rodrigues(rvec1, rmat1, jacobian1);
				cv::Rodrigues(rvec2, rmat2, jacobian2);

				double bott2[1][4] = { {0.0, 0.0, 0.0, 1.0} };
				cv::Mat bott(1, 4, rmat1.type(), &bott2);

				hconcat(rmat1, tvec1, conc1);
				hconcat(rmat2, tvec2, conc2);

				vconcat(conc1, bott, tmat1);
				vconcat(conc2, bott, tmat2);

				cv::cv2eigen(tmat1, tmat_local_left);
				cv::cv2eigen(tmat2, tmat_local_right);
			}
		}

		/****************************************************************
		 * STEP 3: Calculate Local camera position                      *
		 ****************************************************************/
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Base_endeff_left, Base_endeff_right;

		/* Matlab code 
		 * Base_endeff_left = base_leftGlobalCam * leftGlobalCam_pattern * ((leftLocalCam_pattern)^(-1)) * leftLocalCam_endeff */
		Base_endeff_left = base_leftGlobalCam * tmat_global_left * (tmat_local_left.inverse()) * leftLocalCam_endeff;
		position_from_left[i][0] = Base_endeff_left(0, 3);
		position_from_left[i][1] = Base_endeff_left(1, 3);
		position_from_left[i][2] = Base_endeff_left(2, 3);

		/* Matlab code
		 * Base_endeff_left = base_leftGlobalCam * leftGlobalCam_pattern * ((leftLocalCam_pattern)^(-1)) * leftLocalCam_endeff */
		Base_endeff_right = base_rightGlobalCam * tmat_global_right * ((tmat_local_right).inverse()) * rightLocalCam_endeff;
		position_from_right[i][0] = Base_endeff_right(0, 3);
		position_from_right[i][1] = Base_endeff_right(1, 3);
		position_from_right[i][2] = Base_endeff_right(2, 3);
	}
	/****************************************************************
     * STEP 4: Calculate average position                           *
     ****************************************************************/
	double average_left_X = 0, average_left_Y = 0, average_left_Z = 0;
	double average_right_X = 0, average_right_Y = 0, average_right_Z = 0;
	for (int i = 0; i < NR_OF_ITERATIONS; i++)
	{
		average_left_X += position_from_left[i][2];
		average_right_X += position_from_right[i][2];

		average_left_Y += position_from_left[i][0];
		average_right_Y += position_from_right[i][0];

		average_left_Z += position_from_left[i][1];
		average_right_Z += position_from_right[i][1];
	}
	/*
	for (int i = 0; i < NR_OF_ITERATIONS; i++)
	{
		std::cout << "pos left: " << position_from_left[i][0] << " " << position_from_left[i][1] << " " << position_from_left[i][2] << "\n";
	}
	for (int i = 0; i < NR_OF_ITERATIONS; i++)
	{
		std::cout << "pos right: " << position_from_right[i][0] << " " << position_from_right[i][1] << " " << position_from_right[i][2] << "\n";
	}
	*/
	average_left_X = average_left_X / NR_OF_ITERATIONS;
	average_left_Y = average_left_Y / NR_OF_ITERATIONS;
	average_left_Z = average_left_Z / NR_OF_ITERATIONS;

	average_right_X = average_right_X / NR_OF_ITERATIONS;
	average_right_Y = average_right_Y / NR_OF_ITERATIONS;
	average_right_Z = average_right_Z / NR_OF_ITERATIONS;

	std::cout << "Average left X, Y, Z: \n" << average_left_X << " " << average_left_Y << " " << average_left_Z << "\n";
	std::cout << "Average right X, Y, Z: \n" << average_right_X << " " << average_right_Y << " " << average_right_Z << "\n";

	double average_X, average_Y, average_Z;
	average_X = (average_left_X + average_right_X) / 2;
	average_Y = (average_left_Y + average_right_Y) / 2;
	average_Z = (average_left_Z + average_right_Z) / 2;
	std::cout << "\nAverage X, Y, Z: \n" << average_X << " " << average_Y << " " << average_Z << "\n";

	return 1;
}