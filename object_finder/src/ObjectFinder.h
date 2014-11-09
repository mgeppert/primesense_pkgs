#ifndef PRIMESENSE_PKGS_OBJECT_FINDER_H
#define PRIMESENSE_PKGS_OBJECT_FINDER_H

//#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/filters/crop_box.h>
#include <pcl/octree/octree.h>

#define POINTTYPE pcl::PointXYZRGB

namespace primesense_pkgs{

//using pcl::PointCloud;

class ObjectFinder{

public:
    ObjectFinder();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void findObjects();

private:
    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;

    pcl::CropBox<POINTTYPE> lowerBox;
    pcl::CropBox<POINTTYPE> upperBox;
    pcl::octree::OctreePointCloudChangeDetector<POINTTYPE>::Ptr octree;

    pcl::PointCloud<POINTTYPE>::Ptr cropUpperBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropLowerBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr getDifferences(const pcl::PointCloud<POINTTYPE>::Ptr& upc, const pcl::PointCloud<POINTTYPE>::Ptr& lpc);
    //TODO extract objects
};
}//namespace primesense_pkgs

#endif //PRIMESENSE_PKGS_OBJECT_FINDER_H
