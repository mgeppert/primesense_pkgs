#ifndef PRIMESENSE_PKGS_OBJECT_FINDER_H
#define PRIMESENSE_PKGS_OBJECT_FINDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <std_msgs/Time.h>

//PCL
#include <pcl/filters/crop_box.h>
//#include <pcl/octree/octree.h>
//#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

namespace primesense_pkgs{

//using pcl::PointCloud;

class ObjectFinder{

public:
    ObjectFinder();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void findObjects();

private:
    ros::Subscriber sub;
    ros::Publisher objectsPub;
    ros::Publisher upperProjectionPub;
    ros::Publisher lowerProjectionPub;
    ros::Publisher differencesPub;
    ros::Publisher markerPub;

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;
    ros::Time currentCloudTimeStamp;

    pcl::CropBox<POINTTYPE> lowerBox;
    pcl::CropBox<POINTTYPE> upperBox;
    pcl::CropBox<POINTTYPE> smallBox;
    pcl::CropBox<POINTTYPE> triangleBox;
//    pcl::octree::OctreePointCloudChangeDetector<POINTTYPE> octree;

    pcl::PointCloud<POINTTYPE>::Ptr cropUpperBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropLowerBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropTriangleBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
//    pcl::PointCloud<POINTTYPE>::Ptr removeGroundPlane(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr projectToZeroPlane(pcl::PointCloud<POINTTYPE>::Ptr pc);
//    std::vector<int> getDifferenceIndices(const pcl::PointCloud<POINTTYPE>::Ptr& upc, const pcl::PointCloud<POINTTYPE>::Ptr& lpc);
    pcl::PointCloud<POINTTYPE>::Ptr getDifference(const pcl::PointCloud<POINTTYPE>::Ptr& upc, const pcl::PointCloud<POINTTYPE>::Ptr& lpc);
//    pcl::PointCloud<POINTTYPE>::Ptr getDifferenceCloud(const pcl::PointCloud<POINTTYPE>::Ptr& lpc, const std::vector<int>& differenceIndices);
    std::vector<pcl::PointXYZ> getObjectPositions(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    void sendMarker(pcl::PointXYZ point, int id);
};
}//namespace primesense_pkgs

#endif //PRIMESENSE_PKGS_OBJECT_FINDER_H
