#include "TrainingCapture.h"

#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <cstdio>
#include <string>
#include <sstream>

namespace primesense_pkgs{

TrainingCapture::TrainingCapture(){
    ros::NodeHandle nh;
    cloudSub = nh.subscribe("/object_identifier/debug", 1, &TrainingCapture::cloudCallback, this);

    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);

    saveInd = 0;

    return;
}

void TrainingCapture::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *inputCloud);

//    pcl::PointCloud<POINTTYPE>::Ptr toShowCloud = inputCloud->makeShared();

    return;
}

void TrainingCapture::capture(){

    ros::Rate loop_rate(10);

    bool firstRun = true;

    while(ros::ok()){
        ros::spinOnce();

        if(inputCloud->points.size() > 0 && !firstRun){
            saveCloud();
        }
        else if(inputCloud->points.size() > 0){
            firstRun = false;
        }

        loop_rate.sleep();

        if(inputCloud->points.size() > 0){
            std::cin.get();
        }
    }

    return;
}

void TrainingCapture::saveCloud(){

    if(inputCloud->points.size() > 0){
        //translate to origin
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*inputCloud, centroid);
        ROS_INFO("old position: %f, %f, %f", centroid[0], centroid[1], centroid[2]);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << -centroid[0], 0.0, -centroid[2];

        pcl::PointCloud<POINTTYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINTTYPE>());
        pcl::transformPointCloud(*inputCloud, *transformed_cloud, transform);

        Eigen::Vector4d newCentroid;
        pcl::compute3DCentroid(*transformed_cloud, newCentroid);
        ROS_INFO("new position: %f, %f, %f", newCentroid[0], newCentroid[1], newCentroid[2]);

        //save
        std::string fileNum;
        std::string Num = static_cast<std::ostringstream*>( &(std::ostringstream() << saveInd) )->str();
        std::string fileName = "/cloud" + Num + ".pcd";
        std::string filePath = ros::package::getPath("training_capture").append(fileName);
//        pcl::io::savePCDFileASCII (filePath, *transformed_cloud);
        pcl::io::savePCDFileBinary(filePath, *transformed_cloud);
        ROS_INFO("saved current cloud (index %d)", saveInd);

        saveInd++;
    }

    return;
}

}//namespace primesense_pkgs

