////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//		Device Tester Tool								  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "StereoCameraRealSense.h"

#include <map>
#include <thread>
#include <iostream>

#ifdef _WIN32
#include <Windows.h>
inline void do_mkdir(std::string _filename) {
	CreateDirectory(_filename.c_str(), NULL);
}
#elif __linux__ || __APPLE__
#include <sys/stat.h>
#include <sys/types.h>
inline void do_mkdir(std::string _filename) {
	mkdir(_filename.c_str(), 0700);
}
#endif
	

using namespace rgbd;

int imageCounter = 1;
cv::Mat depth, disDepth;
cv::Mat left, right;
std::string rootFolder;
void mouseCallback(int _event, int _x, int _y, int, void*) {
	if (_event == CV_EVENT_LBUTTONUP) {
		cv::imwrite(rootFolder + "/rgb_" + std::to_string(imageCounter) + ".png", left);
		cv::imwrite(rootFolder + "/depth_" + std::to_string(imageCounter) + ".png", depth);
		imageCounter++;
		std::cout << "Saved new pair!" << std::endl;
	}
}

int main(int _argc, char** _argv) {
	rootFolder = "dataset_" + std::to_string(time(NULL));
	do_mkdir(rootFolder);

	StereoCameraRealSense *camera = new StereoCameraRealSense();
	
	if(!camera->init()){
		std::cout << "Failed initialization of the camera" << std::endl;
		return -1;
	}

	cv::namedWindow("left", CV_WINDOW_FREERATIO);
	cv::namedWindow("depth", CV_WINDOW_FREERATIO);
	cv::setMouseCallback("left", mouseCallback);

	while(true){
		// Grab a new frame!
		camera->grab();

		// Show 
		camera->depth(depth);
		cv::normalize(depth, disDepth, 0, 1.0, CV_MINMAX, CV_32F);
		cv::imshow("depth", disDepth);

		camera->rgb(left, right);
		if (left.rows != 0) {
			cv::imshow("left", left);
		}

		cv::waitKey(3);
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));
	return 1;
}
