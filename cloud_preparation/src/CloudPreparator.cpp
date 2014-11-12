#include "CloudPreparator.h"

//#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace primesense_pkgs{

CloudPreparator::CloudPreparator(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/camera/depth_registered/points", 1, &CloudPreparator::cloudCallback, this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_preparation/prepared_cloud", 1);

    cloud = PointCloud<POINTTYPE>::Ptr(new PointCloud<POINTTYPE>);
    return;
}

void CloudPreparator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *cloud);

    return;
}

void CloudPreparator::prepareCloud(){

    PointCloud<POINTTYPE>::Ptr rotatedCloud = adaptViewPoint(cloud);
    PointCloud<POINTTYPE>::Ptr croppedCloud = cropBox(rotatedCloud);
    PointCloud<POINTTYPE>::Ptr filteredCloud = removeOutliers(croppedCloud);
//    PointCloud<POINTTYPE>::Ptr noGroundCloud = removeGroundPlane(rotatedCloud);

    sensor_msgs::PointCloud2 preparedCloudMsg;
    pcl::toROSMsg(*filteredCloud, preparedCloudMsg);
    pub.publish(preparedCloudMsg);

}

PointCloud<POINTTYPE>::Ptr CloudPreparator::adaptViewPoint(const PointCloud<POINTTYPE>::Ptr &cloud){

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

//    // Define a translation of 2.5 meters on the x axis.
//    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; tetha radians arround X axis
//    transform.rotate (Eigen::AngleAxisf ( PI / 4.0, Eigen::Vector3f::UnitX()));

//    float theta = M_PI/4;

    double height = 0.0;
    ros::param::getCached("/calibration/height", height);
    transform.translation() << 0.0, height, 0.0;
    double theta_x = 0.0;
    ros::param::getCached("/calibration/x_angle", theta_x);

    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));

    ROS_INFO("rotate by %f rads around z-axis", M_PI);

    // rotate tetha radians arround X axis
      transform.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));

      ROS_INFO("rotate by %f rads around x-axis", theta_x);



    // Executing the transformation
    pcl::PointCloud<POINTTYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINTTYPE>());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    return transformed_cloud;
}

PointCloud<POINTTYPE>::Ptr CloudPreparator::removeGroundPlane(const PointCloud<POINTTYPE>::Ptr &cloud){

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    PointCloud<POINTTYPE>::Ptr newCloud(new pcl::PointCloud<POINTTYPE>);

    ROS_INFO("points before removing plane: %lu", cloud->points.size());

    pcl::SACSegmentation<POINTTYPE> seg;
//    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f::UnitY());
    seg.setEpsAngle(0.087); //~5 degrees
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.005);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if(inliers->indices.size() > 50){
        ROS_INFO("plane with %lu points detected", inliers->indices.size());
        pcl::ExtractIndices<POINTTYPE> extractor;
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.setInputCloud(cloud);
        extractor.filter(*newCloud);
    }
    else{
        ROS_INFO("not enough points in ground plane, leave");
        newCloud = cloud->makeShared();
    }

    ROS_INFO("points after removing plane: %lu", newCloud->points.size());

    return newCloud;
}

PointCloud<POINTTYPE>::Ptr CloudPreparator::cropBox(const PointCloud<POINTTYPE>::Ptr &cloud){

    pcl::CropBox<POINTTYPE> cb;
    cb.setMin(Eigen::Vector4f(-2.5, -0.1, 0, 1.0));
    cb.setMax(Eigen::Vector4f(2.5, 0.25, 2.5, 1.0));

    cb.setInputCloud(cloud);

    PointCloud<POINTTYPE>::Ptr croppedCloud(new PointCloud<POINTTYPE>);
    cb.filter(*croppedCloud);
    return croppedCloud;

//    double height = 0.0;
//    ros::param::getCached("/calibration/height", height);
//    ROS_INFO("height: %f", height);

//    double theta_x = 0.0;
//    double theta_y = 0.0;
//    double theta_z = 0.0;
//    ros::param::getCached("/calibration/x_angle", theta_x);
//    ros::param::getCached("/calibration/y_angle", theta_y);
//    ros::param::getCached("/calibration/z_angle", theta_z);

//    pcl::CropBox<POINTTYPE> cb;
//    cb.setMin(Eigen::Vector4f(-0.5, -0.1, 0.4, 1.0));
//    cb.setMax(Eigen::Vector4f(0.5, 0.25, 1.5, 1.0));
//    cb.setRotation(Eigen::Vector3f(theta_x, 0.0, M_PI));
//    cb.setTranslation(Eigen::Vector3f(0.0, 0.1, 0.0));
//    cb.setInputCloud(cloud);

//    PointCloud<POINTTYPE>::Ptr croppedCloud(new PointCloud<POINTTYPE>);
//    cb.filter(*croppedCloud);

//    return croppedCloud;
}

PointCloud<POINTTYPE>::Ptr CloudPreparator::removeOutliers(const PointCloud<POINTTYPE>::Ptr& cloud){
    pcl::StatisticalOutlierRemoval<POINTTYPE> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);

    PointCloud<POINTTYPE>::Ptr filteredCloud(new PointCloud<POINTTYPE>);
    sor.filter (*filteredCloud);
    return filteredCloud;
}


}//namespace primesense_pkgs
