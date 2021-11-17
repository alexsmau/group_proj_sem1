#pragma once

#ifndef VISION_MANAGER_H
#define VISION_MANAGER_H

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#define OFFLINE_MODE

class VisionManager
{
private:
	int nr_of_devices;
	std::vector<std::string> serials;
	cv::Size patternsize;

#ifdef OFFLINE_MODE
	std::vector<cv::Mat> left_images;
	std::vector<cv::Mat> right_images;
#endif

	void get_stereo_pair_from_device(int dev_idx, cv::Mat& left_img, cv::Mat& right_img);

public:
	VisionManager();

	/* Return the serial ID of each device, the index of the serial is the index of the device. */
	std::vector<std::string> get_devices_serials();

	/**
	 * Get the pattern onformation from the stereo camera. 
	 * 
	 * [out] img_left       The left image in grayscale. 
	 * [out] corners_left   All the points in the pattern for the left image
	 * [out] img_right      The right image in grayscale. 
	 * [out] corners_right  All the points in the pattern for the right image
	 * [out] map            The mapping between the two set of point vecotrs.
	 * return               True if a checkerboard pattern was found in both images.
	 */
	bool get_patten_info_from_device(int dev_idx, cv::Mat& img_left, std::vector<cv::Point2f>& corners_left, cv::Mat& img_right, std::vector<cv::Point2f>& corners_right, int map[16][2]);

};

#endif