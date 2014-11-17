#include "ObjectIdentifier.h"

#include <object_identifier/Objects.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

namespace primesense_pkgs{

ObjectIdentifier::ObjectIdentifier(){

    ros::NodeHandle nh;
//    cloudSub = nh.subscribe("/cloud_preparation/prepared_cloud", 1, &ObjectIdentifier::cloudCallback, this);
//    positionSub = nh.subscribe("/object_finder/positions", 1, &ObjectIdentifier::positionCallback, this);
    cloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/cloud_preparation/prepared_cloud", 1);
    cloudSub->registerCallback(&ObjectIdentifier::cloudCallback, this);
    positionSub = new message_filters::Subscriber<object_finder::Positions>(nh, "/object_finder/positions", 1);
    positionSub->registerCallback(&ObjectIdentifier::positionCallback, this);
    synchronizer = new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, object_finder::Positions>(*cloudSub, *positionSub, 1000);
    synchronizer->registerCallback(&ObjectIdentifier::cloudPositionCallback, this);

    objectPub = nh.advertise<object_identifier::Objects>("/object_identifier/objects", 1);
    debugPub = nh.advertise<sensor_msgs::PointCloud2>("/object_identifier/debug", 1);

    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);

    return;
}

//ObjectIdentifier::~ObjectIdentifier(){
//    ROS_INFO("destructing ObjectIdentifier");
//    delete cloudSub;
//    delete positionSub;
//    delete synchronizer;
//    ROS_INFO("ObjectIdentifier destructed");
//    return;
//}

void ObjectIdentifier::identifyObjects(){
    extractObjectClouds();
    return;
}

void ObjectIdentifier::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    ROS_INFO("received cloud");

//    pcl::fromROSMsg(*msg, *inputCloud);
    return;
}

void ObjectIdentifier::positionCallback(const object_finder::Positions::ConstPtr &msg){
    ROS_INFO("received positions");

//    objectPositions = std::vector<pcl::PointXYZ>(msg->object_positions.size());

//    for(size_t i = 0; i < msg->object_positions.size(); i++){
//        objectPositions[i].x = msg->object_positions[i].x;
//        objectPositions[i].y = msg->object_positions[i].y;
//        objectPositions[i].z = msg->object_positions[i].z;
//    }

    return;
}

void ObjectIdentifier::cloudPositionCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg, const object_finder::Positions::ConstPtr &posMsg){
    ROS_INFO("received cloud and positions");

    pcl::fromROSMsg(*cloudMsg, *inputCloud);

    objectPositions = std::vector<pcl::PointXYZ>(posMsg->object_positions.size());

    for(size_t i = 0; i < posMsg->object_positions.size(); i++){
        objectPositions[i].x = posMsg->object_positions[i].x;
        objectPositions[i].y = posMsg->object_positions[i].y;
        objectPositions[i].z = posMsg->object_positions[i].z;
    }
    return;
}

void ObjectIdentifier::extractObjectClouds(){

    pcl::CropBox<POINTTYPE> cb;
    cb.setInputCloud(inputCloud);

    objectClouds = std::vector<pcl::PointCloud<POINTTYPE>::Ptr>(objectPositions.size(), pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>));

    for(size_t i = 0; i < objectPositions.size(); i++){
        pcl::PointXYZ op = objectPositions[i];
        cb.setMin(Eigen::Vector4f(op.x - 0.05, 0.0, op.z - 0.05, 1.0));
        cb.setMax(Eigen::Vector4f(op.x + 0.05, 0.05, op.z + 0.05, 1.0));

        cb.filter(*objectClouds[i]);
    }

    if(objectClouds.size() > 0){
        ROS_INFO("sending cloud of first object");
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*objectClouds[0], msg);
        debugPub.publish(msg);
    }
    return;
}
}//namespace primesense_pkgs

