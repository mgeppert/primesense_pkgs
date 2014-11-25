#include <ros/ros.h>

#include "ObjectFinder.h"

int main(int argc, char** argv){
	
	ros::init(argc, argv, "object_finder");

    primesense_pkgs::ObjectFinder objectFinder;

    ros::spin();

//    ros::Rate loop_rate(10);

//    while(ros::ok()){
//        ros::spinOnce();

//        objectFinder.findObjects();

//        loop_rate.sleep();
//    }
	
	return 0;
}
