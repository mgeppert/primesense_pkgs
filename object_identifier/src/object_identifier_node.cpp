#include <ros/ros.h>

#include "ObjectIdentifier.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "object_identifier_node");

    primesense_pkgs::ObjectIdentifier identifier;

    ros::spin();

//    ros::Rate loop_rate(10);

//    TODO: change to callback based
//    while(ros::ok()){
//        ros::spinOnce();
//        identifier.identifyObjects();

//        loop_rate.sleep();
//    }

    return 0;
}
