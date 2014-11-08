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

    int numVals = 0;
    int numIt = 0;
    std::vector<double> averageVals(4, 0.0);

    ros::Rate loop_rate(20);

    while(numVals < 50 && numIt < 1000 && ros::ok()){

        ros::spinOnce();

        numIt++;

        PointCloud<POINTTYPE>::Ptr croppedCloud = cropBox(cloud);
        pcl::ModelCoefficients::Ptr coefficients = findGroundPlane(croppedCloud);

        if(coefficients->values.size() == 0){
            continue;
        }

        ROS_INFO("extracted coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

        for(size_t i = 0; i < 4; i++){
            averageVals[i] += coefficients->values[i];
        }

        numVals++;

        loop_rate.sleep();
    }

    if(numVals < 50){
        //aborted, not enough planes found
        return false;
    }

    for(size_t i = 0; i < 4; i++){
        averageVals[i] /= 50.0;
    }

    ROS_INFO("average vals: %f, %f, %f, %f", averageVals[0], averageVals[1], averageVals[2], averageVals[3]);

    std::vector<double> angles = computeAngles(averageVals);

    ROS_INFO("angles: %f, %f, %f", angles[0], angles[1], angles[2]);

    saveCalibration(angles);

    return true;
}

void Calibrator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

//    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *cloud);

    return;
}

PointCloud<POINTTYPE>::Ptr Calibrator::cropBox(PointCloud<POINTTYPE>::Ptr pc){

    pcl::CropBox<POINTTYPE> cropBox;

    cropBox.setMin(Eigen::Vector4f(-0.2, -1.0, 0.2, 1.0));
    cropBox.setMax(Eigen::Vector4f(0.2, 1.0, 1.5, 1.0));

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
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (pc);
    seg.segment (*inliers, *coefficients);

//    ROS_INFO("size values in fun: %lu", coefficients->values.size());

    return coefficients;
}

std::vector<double> Calibrator::computeAngles(std::vector<double> groundPlaneCoefficients){

    std::vector<double> angles(3);

    //rotation around x axis
//    angles[0] = M_PI_2 - std::atan(-groundPlaneCoefficients[2] / groundPlaneCoefficients[1]);
    angles[0] = std::atan(-groundPlaneCoefficients[2] / groundPlaneCoefficients[1]);
    angles[1] = 0; //rotation around y axis
    angles[2] = 0; //rotation around z axis

    return angles;
}

void Calibrator::saveCalibration(std::vector<double> angles){

    ros::param::set("/calibration/x_angle", angles[0]);
    ros::param::set("/calibration/y_angle", angles[1]);
    ros::param::set("/calibration/z_angle", angles[2]);

    return;
}

}//namespace primesense_pkgs
