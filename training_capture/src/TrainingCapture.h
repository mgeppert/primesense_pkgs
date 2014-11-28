#ifndef PRIMESENSE_PKGS_TRAINING_CAPTURE_H
#define PRIMESENSE_PKGS_TRAINING_CAPTURE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <vector>

#define POINTTYPE pcl::PointXYZ

namespace primesense_pkgs{

class TrainingCapture{

public:
    TrainingCapture();
    void capture();

private:
    ros::Subscriber cloudSub;
    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;
    int saveInd;

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void saveCloud();
};
} //namespace primesense_pkgs

#endif // PRIMESENSE_PKGS_TRAINING_CAPTURE_H
