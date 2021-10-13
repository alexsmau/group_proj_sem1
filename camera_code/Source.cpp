#include<librealsense2/rs.hpp>
#include<iostream>
#include<stdlib.h>
#include <windows.h>
#include<stdio.h>
using namespace std;


// Print the distance
///cout<< "The camera is facing an object ";
//cout<< dist_to_center << " meters away \n";
//int ignorethis2 = printf("The camera is facing an object %f meters away \n", dist_to_center);

int main() {
	int ignorethis = printf("Begin program\n");
	// Create a Pipeline - this serves as a top-level API for streaming and processing frames
	rs2::pipeline p;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);

	// Configure and start the pipeline
	rs2::pipeline_profile pipe_prof = p.start();


	Sleep(3000);

	// Block program until frames arrive
	rs2::frameset frames = p.wait_for_frames();

	// Try to get a frame of a depth image
	rs2::depth_frame depth = frames.get_depth_frame();

	rs2::stream_profile stream = pipe_prof.get_stream(RS2_STREAM_DEPTH);

	rs2::stream_profile stream2 = pipe_prof.get_stream(RS2_STREAM_CONFIDENCE);
	

	rs2::video_stream_profile video_stream = stream.as<rs2::video_stream_profile>();
	rs2_intrinsics intrinsics = video_stream.get_intrinsics();

	rs2::motion_stream_profile device_stream = stream2.as<rs2::motion_stream_profile>();
	rs2_motion_device_intrinsic device_intrinsic = device_stream.get_motion_intrinsics();


	// Get the depth frame's dimensions
	float Width = depth.get_width();
	float height = depth.get_height();


	// Query the distance from the camera to the object in the center of the image
	float dist_to_center = depth.get_distance(Width / 2, height / 2);




	cout << "the intrinsics ppx is = " << intrinsics.ppx << "\n";
	cout << "The camera is facing an object " << dist_to_center << " meters away \n";
	return 0;
}