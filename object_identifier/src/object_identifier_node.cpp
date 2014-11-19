#include <ros/ros.h>

#include "ObjectIdentifier.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "identify_object_node");

    primesense_pkgs::ObjectIdentifier identifier;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        identifier.identifyObjects();

        loop_rate.sleep();
    }

    return 0;
}