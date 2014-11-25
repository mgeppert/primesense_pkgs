#include <ros/ros.h>

#include "RecognitionController.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "recognition_controller");

    primesense_pkgs::RecognitionController controller;

    ros::spin();

	return 0;
}
