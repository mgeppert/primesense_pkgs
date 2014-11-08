#include "CloudPreparator.h"

//#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace primesense_pkgs{

CloudPreparator::CloudPreparator(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/camera/depth_registered/points", 1, &CloudPreparator::cloudCallback, this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_preparation/prepared_cloud", 1);

    cloud = PointCloud<POINTTYPE>::Ptr(new PointCloud<POINTTYPE>);
    return;
}

void CloudPreparator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

//    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *cloud);

    return;
}

void CloudPreparator::prepareCloud(){

    cloud = adaptViewPoint(cloud);

    sensor_msgs::PointCloud2 preparedCloudMsg;
    pcl::toROSMsg(*cloud, preparedCloudMsg);
    pub.publish(preparedCloudMsg);

}

PointCloud<POINTTYPE>::Ptr CloudPreparator::adaptViewPoint(PointCloud<POINTTYPE>::Ptr cloud){

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

//    // Define a translation of 2.5 meters on the x axis.
//    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; tetha radians arround X axis
//    transform.rotate (Eigen::AngleAxisf ( PI / 4.0, Eigen::Vector3f::UnitX()));

//    float theta = M_PI/4;
    double theta_x = 0.0;
    ros::param::getCached("/calibration/x_angle", theta_x);

    // rotate tetha radians arround X axis
      transform.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));

    // Executing the transformation
    pcl::PointCloud<POINTTYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINTTYPE>());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    return transformed_cloud;
}
}//namespace primesense_pkgs
