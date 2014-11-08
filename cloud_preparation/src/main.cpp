#include <ros/ros.h>

#include "CloudPreparator.h"


int main(int argc, char** argv){

    ros::init(argc, argv, "cloud_preparation");

    primesense_pkgs::CloudPreparator preparator;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();

        preparator.prepareCloud();

        loop_rate.sleep();
    }

    return 0;
}
