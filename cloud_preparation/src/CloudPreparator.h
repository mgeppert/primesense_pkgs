#ifndef CLOUD_PREPARATOR_NODE_H
#define CLOUD_PREPARATOR_NODE_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

namespace primesense_pkgs{

using pcl::PointCloud;

class CloudPreparator{

public:
    CloudPreparator();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void prepareCloud();

private:
    std::vector<PointCloud<POINTTYPE>::Ptr> inputClouds;
    PointCloud<POINTTYPE>::Ptr cloud;
    size_t cloudInd;

    ros::Subscriber sub;
    ros::Publisher pub;

    PointCloud<POINTTYPE>::Ptr cropTiltedBox(const PointCloud<POINTTYPE>::Ptr& cloud);
    PointCloud<POINTTYPE>::Ptr adaptViewPoint(const PointCloud<POINTTYPE>::Ptr& cloud);
    PointCloud<POINTTYPE>::Ptr cropBox(const PointCloud<POINTTYPE>::Ptr& cloud);
    PointCloud<POINTTYPE>::Ptr removeOutliers(const PointCloud<POINTTYPE>::Ptr& cloud);

};
}//namespace primesense_pkgs
#endif // CLOUD_PREPARATOR_NODE_H
