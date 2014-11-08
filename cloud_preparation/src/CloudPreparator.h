#ifndef CLOUD_PREPARATION_NODE_H
#define CLOUD_PREPARATION_NODE_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define POINTTYPE pcl::PointXYZRGB

namespace primesense_pkgs{

using pcl::PointCloud;

class CloudPreparator{

public:
    CloudPreparator();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void prepareCloud();

private:

//    static const double PI = 3.1415926535;;

    PointCloud<POINTTYPE>::Ptr cloud;

    ros::Subscriber sub;
    ros::Publisher pub;

    PointCloud<POINTTYPE>::Ptr adaptViewPoint(PointCloud<POINTTYPE>::Ptr cloud);
};

}

#endif // CLOUD_PREPARATION_NODE_H
