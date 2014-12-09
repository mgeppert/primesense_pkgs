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
//    static bool positionCompare(pcl::PointXYZ lhs, pcl::PointXYZ rhs);


private:
    ros::Subscriber sub;
    ros::Publisher objectsPub;
    ros::Publisher upperProjectionPub;
    ros::Publisher lowerProjectionPub;
    ros::Publisher differencesPub;
    ros::Publisher markerPub;
    ros::Publisher wallPointsPub;

//    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;
//    ros::Time currentCloudTimeStamp;

    pcl::CropBox<POINTTYPE> lowerBox;
    pcl::CropBox<POINTTYPE> upperBox;
    pcl::CropBox<POINTTYPE> smallBox;
    pcl::CropBox<POINTTYPE> triangleBox;

    struct objectPose{
        pcl::PointXYZ position;
        double angle;
    };

    pcl::PointCloud<POINTTYPE>::Ptr adaptViewPoint(const pcl::PointCloud<POINTTYPE>::Ptr& cloud);
    void findObjects(const pcl::PointCloud<POINTTYPE>::Ptr &inputCloud, ros::Time currentCloudTimeStamp);
    pcl::PointCloud<POINTTYPE>::Ptr downSample(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropUpperBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropLowerBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr cropTriangleBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr projectToZeroPlane(pcl::PointCloud<POINTTYPE>::Ptr pc);
    pcl::PointCloud<POINTTYPE>::Ptr getDifference(const pcl::PointCloud<POINTTYPE>::Ptr& upc, const pcl::PointCloud<POINTTYPE>::Ptr& lpc);
    std::vector<ObjectFinder::objectPose> getObjectPoses(const pcl::PointCloud<POINTTYPE>::Ptr& pc);
    pcl::PointCloud<POINTTYPE>::Ptr removeOutliers(const pcl::PointCloud<POINTTYPE>::Ptr& cloud);
    static bool positionCompare(const ObjectFinder::objectPose& lhs, const ObjectFinder::objectPose& rhs);
    void sendMarker(pcl::PointXYZ point, int id, ros::Time timestamp);
    void sendWallPoints(const pcl::PointCloud<POINTTYPE>::Ptr& pc, ros::Time timestamp);
};
}//namespace primesense_pkgs

#endif //PRIMESENSE_PKGS_OBJECT_FINDER_H
