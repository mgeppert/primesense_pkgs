#include <ros/ros.h>

#include "TrainingCapture.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "training_capture_node");

    primesense_pkgs::TrainingCapture capturer;

    capturer.capture();

    return 0;
}
