#include "VisionManager.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types_c.h>

#include <librealsense2/rs.hpp>

void VisionManager::init_devices()
{
	dev_list = ctx.query_devices();
	
	serials.clear();
	for (rs2::device dev : dev_list)
	{
		printf("Device serial nr is %s \n", dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	}

	nr_of_devices = serials.size();
}

void VisionManager::configure_IR_projector()
{
	for (rs2::device dev : dev_list)
	{
		// Given a device, we can query its sensors using:
		std::vector<rs2::sensor> sensors = dev.query_sensors();

		//printf("Device consists of %d ensors:\n", (int)sensors.size());

		/* Disable the IR sensor. We do not want the dotted pattern. */
		rs2::sensor stereo_sensor = sensors[0];
		stereo_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
	}
}

void VisionManager::configure_video_stream()
{
	pipelines.clear();
	for (int i = 0; i < nr_of_devices; i++)
	{
		pipelines.push_back(rs2::pipeline(ctx));
		rs2::config cfg;
		cfg.enable_stream(STREAM, STREAM_INDEX_LEFT, WIDTH, HEIGHT, FORMAT, FPS);
		cfg.enable_stream(STREAM, STREAM_INDEX_RIGHT, WIDTH, HEIGHT, FORMAT, FPS);
		cfg.enable_device(serials[i]);
		pipelines[i].start(cfg);
	}
}

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
	init_devices();
	configure_IR_projector();
	configure_video_stream();
#endif
}

void VisionManager::get_stereo_pair_from_device(int dev_idx, cv::Mat& left_img, cv::Mat& right_img)
{
#ifdef OFFLINE_MODE
	left_images[dev_idx].copyTo(left_img);
	right_images[dev_idx].copyTo(right_img);
#else
	printf("\n\n!!!!!!!!!!!!!GET PAIR!!!!!!!!!!!!\n");
	rs2::frameset frames = pipelines[dev_idx].wait_for_frames();
	printf("There are %d frames found\n", (int)frames.size());
	rs2::video_frame left_frame = frames.get_infrared_frame(STREAM_INDEX_LEFT);
	rs2::video_frame right_frame = frames.get_infrared_frame(STREAM_INDEX_RIGHT);

	cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, (void*)left_frame.get_data()).copyTo(left_img);
	cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, (void*)right_frame.get_data()).copyTo(right_img);
#endif

}

bool VisionManager::get_patten_info_from_device(int dev_idx, cv::Mat& img_left, std::vector<cv::Point2f>& corners_left, cv::Mat& img_right, std::vector<cv::Point2f>& corners_right, int map[16][2])
{
	corners_left.clear();
	corners_right.clear();

	get_stereo_pair_from_device(dev_idx, img_left, img_right);
	/*
	cv::namedWindow("img_left", cv::WINDOW_NORMAL);
	imshow("img_left", img_left);
	cv::namedWindow("img_right", cv::WINDOW_NORMAL);
	imshow("img_right", img_right);
	cv::waitKey(0);
	*/
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

	/*
	drawChessboardCorners(img_left, patternsize, cv::Mat(corners_left), left_patternfound);
	cv::namedWindow("left_with_corners", cv::WINDOW_NORMAL);
	imshow("left_with_corners", img_left);
	cv::waitKey(0);
	*/

	/* Currently the order is maintained between the two corner sets. I am not sure what happens if the image is very tilted.
	 * I will have to experiment a bit more with this. */
	for (int i = 0; i < 16; i++)
	{
		map[i][0] = i;
		map[i][1] = i;
	}

	return true;
}

rs2_intrinsics VisionManager::get_intrinsics_from_device(int dev_idx)
{
	rs2::stream_profile stream = pipelines[dev_idx].get_active_profile().get_stream(STREAM);
	rs2::video_stream_profile video_stream = stream.as<rs2::video_stream_profile>();
	return video_stream.get_intrinsics();
}