#include "VisionManager.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types_c.h>

VisionManager::VisionManager()
{
	patternsize = cv::Size(4, 4);

#ifdef OFFLINE_MODE
	nr_of_devices = 1;
	serials.push_back("OFFLINE_MODE");

	left_images.clear();
	right_images.clear();
	cv::Mat left_img = cv::imread("..\\..\\group_proj_sem1\\camera_code\\sample_images\\left_img1.png", cv::IMREAD_GRAYSCALE);
	if (left_img.empty())
	{
		printf("Could not open or find pattern_left.png\n");
		cv::waitKey(0); //wait for any key press
	}
	left_images.push_back(left_img);
	left_images.push_back(cv::imread("..\\..\\group_proj_sem1\\camera_code\\sample_images\\left_img2.png", cv::IMREAD_GRAYSCALE));
	left_images.push_back(cv::imread("..\\..\\group_proj_sem1\\camera_code\\sample_images\\left_img3.png", cv::IMREAD_GRAYSCALE));

	right_images.push_back(cv::imread("..\\..\\group_proj_sem1\\camera_code\\sample_images\\right_img1.png", cv::IMREAD_GRAYSCALE));
	right_images.push_back(cv::imread("..\\..\\group_proj_sem1\\camera_code\\sample_images\\right_img2.png", cv::IMREAD_GRAYSCALE));
	right_images.push_back(cv::imread("..\\..\\group_proj_sem1\\camera_code\\sample_images\\right_img3.png", cv::IMREAD_GRAYSCALE));

#else

#endif
}

void VisionManager::get_stereo_pair_from_device(int dev_idx, cv::Mat& left_img, cv::Mat& right_img)
{
#ifdef OFFLINE_MODE
	left_images[dev_idx].copyTo(left_img);
	right_images[dev_idx].copyTo(right_img);
#else

#endif // DEBUG

}

bool VisionManager::get_patten_info_from_device(int dev_idx, cv::Mat& img_left, std::vector<cv::Point2f>& corners_left, cv::Mat& img_right, std::vector<cv::Point2f>& corners_right, int map[16][2])
{
	corners_left.clear();
	corners_right.clear();

	get_stereo_pair_from_device(dev_idx, img_left, img_right);

	bool left_patternfound = findChessboardCorners(img_left, patternsize, corners_left,  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
	if (!left_patternfound)
	{
		return false;
	}

	bool right_patternfound = findChessboardCorners(img_right, patternsize, corners_right, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
	if (!right_patternfound)
	{
		return false;
	}

	cv::cornerSubPix(img_left, corners_left, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	cv::cornerSubPix(img_right, corners_right, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	/* Currently the order is maintained between the two corner sets. I am not sure what happens if the image is very tilted.
	 * I will have to experiment a bit more with this. */
	for (int i = 0; i < 16; i++)
	{
		map[i][0] = i;
		map[i][1] = i;
	}

	return true;
}