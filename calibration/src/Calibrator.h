#ifndef PRIMESENSE_PKGS_CALIBRATOR_H
#define PRIMESENSE_PKGS_CALIBRATOR_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <vector>

#define POINTTYPE pcl::PointXYZ

namespace primesense_pkgs{

using pcl::PointCloud;

class Calibrator{

public:
    Calibrator();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool calibrate();

private:
    PointCloud<POINTTYPE>::Ptr cloud;

    ros::Subscriber sub;

    PointCloud<POINTTYPE>::Ptr cropBox(PointCloud<POINTTYPE>::Ptr pc);
    pcl::ModelCoefficients::Ptr findGroundPlane(PointCloud<POINTTYPE>::Ptr pc);
    std::vector<double> computeAngles(std::vector<float> groundPlaneCoefficients);
    void saveCalibration(std::vector<double> angles, double height);

};
} //namespace primesense_pkgs

#endif //PRIMESENSE_PKGS_CALIBRATOR_H
