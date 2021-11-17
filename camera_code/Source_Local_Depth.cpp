#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <stdio.h>
#include <librealsense2/rs.hpp>
#include <vector>


using namespace cv;
using namespace std;

void depth_calc(float k1_x, float k1_y, float k2_x, float k2_y) {

	//!this focal length is not based on our intrinsics! :-(
	float focal_length = 1.88;
	int baseline = 50;

	float x_disp = k1_x - k2_x;
	float y_disp = k1_y - k2_y;

	vector<float> disparity;

	disparity.push_back(x_disp);
	disparity.push_back(y_disp);

	cout << "disparity vector = " << disparity[0] << ", " << disparity[1] << endl;

	float x_depth = (focal_length * baseline) / disparity[0]; 
	float y_depth = (focal_length * baseline) / disparity[1];

	vector<float> depth; 

	depth.push_back(x_depth); 
	depth.push_back(y_depth);

	cout << "depth vector = " << depth[0] << ", " << depth[1] << endl;
}


void main() {

	depth_calc(1,1,10,10);
	
}



