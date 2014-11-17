#ifndef PRIMESENSE_PKGS_OBJECT_IDENTIFIER_H
#define PRIMESENSE_PKGS_OBJECT_IDENTIFIER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_finder/Positions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

namespace primesense_pkgs{

class ObjectIdentifier
{
public:
    ObjectIdentifier();
//    ~ObjectIdentifier();
    void identifyObjects();

private:
//    ros::Subscriber cloudSub;
//    ros::Subscriber positionSub;
    ros::Publisher objectPub;
    ros::Publisher debugPub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloudSub;
    message_filters::Subscriber<object_finder::Positions> *positionSub;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, object_finder::Positions> *synchronizer;

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;

    std::vector<pcl::PointXYZ> objectPositions;
    std::vector<pcl::PointCloud<POINTTYPE>::Ptr> objectClouds;

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void positionCallback(const object_finder::Positions::ConstPtr &msg);
    void cloudPositionCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg, const object_finder::Positions::ConstPtr &posMsg);
    void extractObjectClouds();

};
} //namespace primesense_pkgs

#endif // PRIMESENSE_PKGS_OBJECT_IDENTIFIER_H
