#ifndef PRIMESENSE_PKGS_OBJECT_FINDER_H
#define PRIMESENSE_PKGS_OBJECT_FINDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/filters/crop_box.h>

#include <vector>

#define POINTTYPE pcl::PointXYZ

namespace primesense_pkgs{

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

    pcl::PointCloud<POINTTYPE>::Ptr cropUpperBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropLowerBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropTriangleBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr projectToZeroPlane(pcl::PointCloud<POINTTYPE>::Ptr pc);
    pcl::PointCloud<POINTTYPE>::Ptr getDifference(const pcl::PointCloud<POINTTYPE>::Ptr& upc, const pcl::PointCloud<POINTTYPE>::Ptr& lpc);
    std::vector<pcl::PointXYZ> getObjectPositions(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    void sendMarker(pcl::PointXYZ point, int id);
};
}//namespace primesense_pkgs

#endif //PRIMESENSE_PKGS_OBJECT_FINDER_H
