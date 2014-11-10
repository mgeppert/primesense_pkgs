#include "ObjectFinder.h"

#include <object_finder/Objects.h>

#include <pcl_conversions/pcl_conversions.h>

namespace primesense_pkgs{

ObjectFinder::ObjectFinder(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/camera/depth_registered/points", 1, &ObjectFinder::cloudCallback, this);
    objectsPub = nh.advertise<object_finder::Objects>("object_finder/objects", 1);
    upperProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("object_finder/upper_projection", 1);
    lowerProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("object_finder/lower_projection", 1);
    differencesPub = nh.advertise<sensor_msgs::PointCloud2>("object_finder/differences", 1);

    lowerBox = pcl::CropBox<POINTTYPE>();
    lowerBox.setMin(Eigen::Vector4f(-10.0, 0.02, 0.0, 1.0));
    lowerBox.setMax(Eigen::Vector4f(10.0, 0.1, 10.0, 1.0));

    upperBox = pcl::CropBox<POINTTYPE>();
    upperBox.setMin(Eigen::Vector4f(-10.0, 0.1, 0.0, 1.0));
    upperBox.setMax(Eigen::Vector4f(10.0, 0.2, 10.0, 1.0));

    float resolution = 32.0f;
//    octree = pcl::octree::OctreePointCloudChangeDetector<POINTTYPE>::Ptr(new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>(resolution));

    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);
    currentCloudTimeStamp = ros::Time::now();


    return;
}

void ObjectFinder::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
//    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *inputCloud);
    return;
}

void ObjectFinder::findObjects(){

    pcl::PointCloud<POINTTYPE>::Ptr upperCloud = cropUpperBox(inputCloud);
    pcl::PointCloud<POINTTYPE>::Ptr lowerCloud = cropLowerBox(inputCloud);

    pcl::PointCloud<POINTTYPE>::Ptr upperProjection = projectToZeroPlane(upperCloud);
    pcl::PointCloud<POINTTYPE>::Ptr lowerProjection = projectToZeroPlane(lowerCloud);

    sensor_msgs::PointCloud2 upperProjectionMsg;
    pcl::toROSMsg(*upperProjection, upperProjectionMsg);
    upperProjectionPub.publish(upperProjectionMsg);

    sensor_msgs::PointCloud2 lowerProjectionMsg;
    pcl::toROSMsg(*lowerProjection, lowerProjectionMsg);
    upperProjectionPub.publish(lowerProjectionMsg);

    std::vector<int> differenceIndices = getDifferenceIndices(upperProjection, lowerProjection);

    pcl::PointCloud<POINTTYPE>::Ptr differenceCloud = getDifferenceCloud(lowerCloud, differenceIndices);

    sensor_msgs::PointCloud2 differenceCloudMsg;
    pcl::toROSMsg(*differenceCloud, differenceCloudMsg);
    differenceCloudMsg.header = std_msgs::Header();
    differenceCloudMsg.header.stamp = currentCloudTimeStamp;
    differencesPub.publish(differenceCloudMsg);

    return;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::cropUpperBox(const pcl::PointCloud<POINTTYPE>::Ptr &pc){
    upperBox.setInputCloud(pc);

    pcl::PointCloud<POINTTYPE>::Ptr croppedCloud(new pcl::PointCloud<POINTTYPE>());
    upperBox.filter(*croppedCloud);

    return croppedCloud;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::cropLowerBox(const pcl::PointCloud<POINTTYPE>::Ptr &pc){
    lowerBox.setInputCloud(pc);

    pcl::PointCloud<POINTTYPE>::Ptr croppedCloud(new pcl::PointCloud<POINTTYPE>());
    lowerBox.filter(*croppedCloud);

    return croppedCloud;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::projectToZeroPlane(pcl::PointCloud<POINTTYPE>::Ptr pc){

    pcl::PointCloud<POINTTYPE>::Ptr projectedCloud = pc->makeShared();

    for(size_t i = 0; i < projectedCloud->points.size(); i++){
        if(!isnan(projectedCloud->points[i].y)){
            pc->points[i].y = 0;
        }
    }
    return projectedCloud;
}

std::vector<int> ObjectFinder::getDifferenceIndices(const pcl::PointCloud<POINTTYPE>::Ptr &upc, const pcl::PointCloud<POINTTYPE>::Ptr &lpc){

    float resolution = 32.0f;
    pcl::octree::OctreePointCloudChangeDetector<POINTTYPE> octree(resolution);

    octree.setInputCloud(upc);
    octree.addPointsFromInputCloud();

    octree.switchBuffers();

    octree.setInputCloud(lpc);
    octree.addPointsFromInputCloud();

    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    return newPointIdxVector;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::getDifferenceCloud(const pcl::PointCloud<POINTTYPE>::Ptr &lpc, const std::vector<int> &differenceIndices){

    pcl::PointCloud<POINTTYPE>::Ptr differenceCloud(new pcl::PointCloud<POINTTYPE>);
    differenceCloud->points.resize(differenceIndices.size());

    for(size_t i = 0; i < differenceIndices.size(); i++){
        differenceCloud->points[i] = lpc->points[differenceIndices[i]];
    }
    return differenceCloud;
}

} //namespace primesense_pkgs
