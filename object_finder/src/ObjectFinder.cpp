#include "ObjectFinder.h"

#include <ros/ros.h>

namespace primesense_pkgs{

ObjectFinder::ObjectFinder(){
    //TODO
    return;
}

void ObjectFinder::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
//    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *inputCloud);
    return;
}

void ObjectFinder::findObjects(){
    //TODO
    return;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::cropUpperBox(const pcl::PointCloud<POINTTYPE>::Ptr &pc){
    //TODO
    return pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>); //CHANGE!!!
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::cropLowerBox(const pcl::PointCloud<POINTTYPE>::Ptr &pc){
    //TODO
    return pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>); //CHANGE!!!
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::getDifferences(const pcl::PointCloud<POINTTYPE>::Ptr &upc, const pcl::PointCloud<POINTTYPE>::Ptr &lpc){
    //TODO
    return pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>); //CHANGE!!!
}

} //namespace primesense_pkgs
