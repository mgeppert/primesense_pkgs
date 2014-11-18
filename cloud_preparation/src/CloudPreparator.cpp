#include "CloudPreparator.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/plane_refinement_comparator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace primesense_pkgs{

CloudPreparator::CloudPreparator(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/camera/depth_registered/points", 1, &CloudPreparator::cloudCallback, this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_preparation/prepared_cloud", 1);

    inputClouds = std::vector<PointCloud<POINTTYPE>::Ptr>(2, PointCloud<POINTTYPE>::Ptr(new PointCloud<POINTTYPE>));
    cloud = PointCloud<POINTTYPE>::Ptr(new PointCloud<POINTTYPE>);
    cloudInd = 0;
    return;
}

void CloudPreparator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    ROS_INFO("received cloud");

//    PointCloud<POINTTYPE>::Ptr tempCloud(new PointCloud<POINTTYPE>);
    pcl::fromROSMsg(*msg, *inputClouds[cloudInd]);
    *cloud = *inputClouds[0] + *inputClouds[1];
    cloudInd = (cloudInd + 1) % 2;

    return;
}

void CloudPreparator::prepareCloud(){

    PointCloud<POINTTYPE>::Ptr cCloud = cropTiltedBox(cloud);
    PointCloud<POINTTYPE>::Ptr rotatedCloud = adaptViewPoint(cCloud);
    PointCloud<POINTTYPE>::Ptr croppedCloud = cropBox(rotatedCloud);
//    PointCloud<POINTTYPE>::Ptr filteredCloud = removeOutliers(croppedCloud);

    sensor_msgs::PointCloud2 preparedCloudMsg;
    pcl::toROSMsg(*croppedCloud, preparedCloudMsg);
    pub.publish(preparedCloudMsg);

}

PointCloud<POINTTYPE>::Ptr CloudPreparator::cropTiltedBox(const PointCloud<POINTTYPE>::Ptr &cloud){

    double theta_x = 0.0;
    double theta_y = 0.0;
    double theta_z = 0.0;
    ros::param::getCached("/calibration/x_angle", theta_x);
    ros::param::getCached("/calibration/y_angle", theta_y);
    ros::param::getCached("/calibration/z_angle", theta_z);

    pcl::CropBox<POINTTYPE> cb;
    cb.setMin(Eigen::Vector4f(-0.5, -0.5, 0.0, 1.0));
    cb.setMax(Eigen::Vector4f(0.5, 0.25, 2.0l, 1.0));
    cb.setRotation(Eigen::Vector3f(theta_x, 0.0, M_PI));
    cb.setInputCloud(cloud);

    PointCloud<POINTTYPE>::Ptr croppedCloud(new PointCloud<POINTTYPE>);
    cb.filter(*croppedCloud);

    return croppedCloud;
}

PointCloud<POINTTYPE>::Ptr CloudPreparator::adaptViewPoint(const PointCloud<POINTTYPE>::Ptr &cloud){

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    double height = 0.0;
    ros::param::getCached("/calibration/height", height);
    transform.translation() << 0.0, height, 0.0;
    double theta_x = 0.0;
    ros::param::getCached("/calibration/x_angle", theta_x);

    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));

    // rotate tetha radians arround X axis
    transform.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));

    ROS_INFO("rotate by %f rads around x-axis", theta_x);

    // Executing the transformation
    pcl::PointCloud<POINTTYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINTTYPE>());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    return transformed_cloud;
}

PointCloud<POINTTYPE>::Ptr CloudPreparator::cropBox(const PointCloud<POINTTYPE>::Ptr &cloud){

    pcl::CropBox<POINTTYPE> cb;
    cb.setMin(Eigen::Vector4f(-2.5, 0.01, 0, 1.0));
    cb.setMax(Eigen::Vector4f(2.5, 0.25, 2.5, 1.0));

    cb.setInputCloud(cloud);

    PointCloud<POINTTYPE>::Ptr croppedCloud(new PointCloud<POINTTYPE>);
    cb.filter(*croppedCloud);
    return croppedCloud;
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
