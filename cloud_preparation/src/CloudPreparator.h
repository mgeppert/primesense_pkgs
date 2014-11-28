#ifndef CLOUD_PREPARATOR_NODE_H
#define CLOUD_PREPARATOR_NODE_H

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

    PointCloud<POINTTYPE>::Ptr cloud;

    ros::Subscriber sub;
    ros::Publisher pub;

    PointCloud<POINTTYPE>::Ptr cropTiltedBox(const PointCloud<POINTTYPE>::Ptr& cloud);
    PointCloud<POINTTYPE>::Ptr adaptViewPoint(const PointCloud<POINTTYPE>::Ptr& cloud);
    PointCloud<POINTTYPE>::Ptr cropBox(const PointCloud<POINTTYPE>::Ptr& cloud);
    PointCloud<POINTTYPE>::Ptr removeOutliers(const PointCloud<POINTTYPE>::Ptr& cloud);

};
}//namespace primesense_pkgs
#endif // CLOUD_PREPARATOR_NODE_H
