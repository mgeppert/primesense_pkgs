#include <ros/ros.h>

#include "Calibrator.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "calibration_node");

    primesense_pkgs::Calibrator calibrator;

    // loop is just for debugging - remove!!

    if(calibrator.calibrate()){
        ROS_INFO("calibration successful");
    }
    else{
        ROS_INFO("calibration failed");
    }

    return 0;
}
