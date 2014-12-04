#include <ros/ros.h>

#include "ImageBuffer.h"

int main(int argc, char** argv){
	
	ros::init(argc, argv, "image_buffer");
	
	primesense_pkgs::ImageBuffer buffer;
	
	ros::Rate loop_rate(3);
	
	while(ros::ok()){
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	return 0;
}
