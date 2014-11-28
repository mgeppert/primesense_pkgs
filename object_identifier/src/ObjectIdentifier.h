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

private:
    ros::Publisher objectPub;
    ros::Publisher debugPub;
    ros::Publisher speakerPub;
    ros::Time currentObjectsTimestamp;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloudSub;
    message_filters::Subscriber<object_finder::Positions> *positionSub;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, object_finder::Positions> *synchronizer;

    std::vector<std::string> trainingSampleLabels;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > trainingSamples;
//    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;
//    std::vector<pcl::PointXYZ> objectPositions;
//    std::vector<double> objectRotations;
//    std::vector<pcl::PointCloud<POINTTYPE>::Ptr> objectClouds;


    void identifyObjects(std::vector<pcl::PointXYZ> objectPositions, std::vector<double> objectRotations, const pcl::PointCloud<POINTTYPE>::Ptr &inputCloud);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void positionCallback(const object_finder::Positions::ConstPtr &msg);
    void cloudPositionCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg, const object_finder::Positions::ConstPtr &posMsg);
    std::vector<pcl::PointCloud<POINTTYPE>::Ptr> extractObjectClouds(const std::vector<pcl::PointXYZ> &objectPositions, const std::vector<double> &objectRotations, const pcl::PointCloud<POINTTYPE>::Ptr &inputCloud);
    bool loadTrainingData(std::vector<std::string>& labelNames, std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> >& trainingSamples);
    std::string identifySingleObject(const pcl::PointCloud<POINTTYPE>::Ptr& object, std::string color);
    void removeDuplicatePositions(std::vector<pcl::PointXYZ> &objectPositions, std::vector<double> &objectRotations);
    std::string getObjectColor(const pcl::PointCloud<POINTTYPE>::Ptr& object);
    std::string classifyColor(double h, double s, double v);
    void publishFoundObjects(const std::vector<std::string>& colors, const std::vector<std::string>& shapes, const std::vector<pcl::PointXYZ>& positions);
    std::vector<int> getPossibleShapeIndices(std::string color, const std::vector<std::string> &shapes);

};
} //namespace primesense_pkgs

#endif // PRIMESENSE_PKGS_OBJECT_IDENTIFIER_H
