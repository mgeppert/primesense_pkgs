#include "Calibrator.h"

#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cmath>

namespace primesense_pkgs{

Calibrator::Calibrator(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/camera/depth_registered/points", 1, &Calibrator::cloudCallback, this);

    cloud = PointCloud<POINTTYPE>::Ptr(new PointCloud<POINTTYPE>);

//    ROS_INFO("constructed");

    return;
}

bool Calibrator::calibrate(){

//    ROS_INFO("points before cropbox: %lu", cloud->points.size());

//    PointCloud<POINTTYPE>::Ptr croppedCloud = cropBox(cloud);
//    ROS_INFO("cropped (%lu points left)", croppedCloud->points.size());
//    pcl::ModelCoefficients::Ptr coefficients = findGroundPlane(croppedCloud);

//    if(coefficients->values.size() == 0){
//        return false;
//    }

//    ROS_INFO("values size: %lu", coefficients->values.size());

//    ROS_INFO("extracted coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    int requiredClouds = 50;

    int numVals = 0;
    int numIt = 0;
    std::vector<double> averageAngles(3, 0.0);
    std::vector<std::vector<double> > allAngles(requiredClouds, std::vector<double>(3, 0));
    double averageHeight = 0.0;
    std::vector<double> allHeights(requiredClouds, 0);

    ros::Rate loop_rate(20);

    while(numVals < requiredClouds && numIt < 1000 && ros::ok()){

        ros::spinOnce();

        numIt++;

        if(cloud->points.size() == 0){
            loop_rate.sleep();
            continue;
        }

        PointCloud<POINTTYPE>::Ptr croppedCloud = cropBox(cloud);
        pcl::ModelCoefficients::Ptr coefficients = findGroundPlane(croppedCloud);

        if(coefficients->values.size() == 0){
            loop_rate.sleep();
            continue;
        }

        ROS_INFO("extracted coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

        std::vector<double> angles = computeAngles(coefficients->values);

        for(size_t i = 0; i < 3; i++){

            averageAngles[i] += angles[i];
            allAngles[numVals][i] = angles[i];
        }

//        averageHeight += -coefficients->values[1]*coefficients->values[3];
        averageHeight += (double) coefficients->values[3]/ (double) coefficients->values[2] * std::sin(angles[0]);
//        allHeights[numVals] = -coefficients->values[1]*coefficients->values[3];
        allHeights[numVals] = (double) coefficients->values[3]/ (double) coefficients->values[2] * std::sin(angles[0]);

        numVals++;

        loop_rate.sleep();
    }

    if(numVals < requiredClouds){
        //aborted, not enough planes found
        return false;
    }

    //print x-angles + heights
    for(size_t i = 0; i < allAngles.size(); i++){
        ROS_INFO("x_ang[%lu]: %f, height: %f", i, allAngles[i][0], allHeights[i]);
    }

    for(size_t i = 0; i < 3; i++){
        averageAngles[i] /= requiredClouds;
    }
    averageHeight /= requiredClouds;

    ROS_INFO("average angles: %f, %f, %f, %f", averageAngles[0], averageAngles[1], averageAngles[2], averageAngles[3]);

    ROS_INFO("average height: %f", averageHeight);

    saveCalibration(averageAngles, averageHeight);

    return true;
}

void Calibrator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *cloud);

    return;
}

PointCloud<POINTTYPE>::Ptr Calibrator::cropBox(PointCloud<POINTTYPE>::Ptr pc){

    pcl::CropBox<POINTTYPE> cropBox;

    cropBox.setMin(Eigen::Vector4f(-0.1, -0.2, 0.2, 1.0));
    cropBox.setMax(Eigen::Vector4f(0.1, 0.5, 1.0, 1.0));

    cropBox.setInputCloud(pc);

    pcl::PointCloud<POINTTYPE>::Ptr croppedCloud(new pcl::PointCloud<POINTTYPE>());
    cropBox.filter(*croppedCloud);

    return croppedCloud;
}

pcl::ModelCoefficients::Ptr Calibrator::findGroundPlane(PointCloud<POINTTYPE>::Ptr pc){

//    return pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients); //CHANGE!!!

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<POINTTYPE> seg;
    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold (0.001);

    seg.setInputCloud (pc);
    seg.segment (*inliers, *coefficients);

//    ROS_INFO("size values in fun: %lu", coefficients->values.size());

    return coefficients;
}

std::vector<double> Calibrator::computeAngles(std::vector<float> groundPlaneCoefficients){

    std::vector<double> angles(3);

    //rotation around x axis
//    angles[0] = M_PI_2 - std::atan(-groundPlaneCoefficients[2] / groundPlaneCoefficients[1]);
//    angles[0] = std::atan(-groundPlaneCoefficients[2] / groundPlaneCoefficients[1]);
    angles[0] = -std::atan((double) groundPlaneCoefficients[2] / (double) groundPlaneCoefficients[1]);
    angles[1] = 0; //rotation around y axis
    angles[2] = 0; //rotation around z axis

    return angles;
}

void Calibrator::saveCalibration(std::vector<double> angles, double height){

    ros::param::set("/calibration/x_angle", angles[0]);
    ros::param::set("/calibration/y_angle", angles[1]);
    ros::param::set("/calibration/z_angle", angles[2]);
    ros::param::set("/calibration/height", height);

    return;
}

}//namespace primesense_pkgs
