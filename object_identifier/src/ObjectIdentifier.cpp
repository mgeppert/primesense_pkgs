#include "ObjectIdentifier.h"

#include <object_identifier/Objects.h>

#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

#include <fstream>

namespace primesense_pkgs{

ObjectIdentifier::ObjectIdentifier(){

    ros::NodeHandle nh;
    cloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/cloud_preparation/prepared_cloud", 1);
    cloudSub->registerCallback(&ObjectIdentifier::cloudCallback, this);
    positionSub = new message_filters::Subscriber<object_finder::Positions>(nh, "/object_finder/positions", 1);
    positionSub->registerCallback(&ObjectIdentifier::positionCallback, this);
    synchronizer = new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, object_finder::Positions>(*cloudSub, *positionSub, 1000);
    synchronizer->registerCallback(&ObjectIdentifier::cloudPositionCallback, this);

    objectPub = nh.advertise<object_identifier::Objects>("/object_identifier/objects", 1);
    debugPub = nh.advertise<sensor_msgs::PointCloud2>("/object_identifier/debug", 1);

    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);

    if(!loadTrainingData(trainingSampleLabels, trainingSamples)){
        ROS_ERROR("unable to read training data");
    }

    return;
}

void ObjectIdentifier::identifyObjects(){
    extractObjectClouds();

    for(size_t i = 0; i < objectClouds.size(); i++){

        //TODO: check color??
        std::string shape = identifySingleObject(objectClouds[i]);

        ROS_INFO("%s at position (%f, %f)", shape.c_str(), objectPositions[i].x, objectPositions[i].z);
    }
    return;
}

void ObjectIdentifier::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    //just for easier debugging
    ROS_INFO("received cloud");
    return;
}

void ObjectIdentifier::positionCallback(const object_finder::Positions::ConstPtr &msg){
    //just for easier debugging
    ROS_INFO("received positions");
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

bool ObjectIdentifier::loadTrainingData(std::vector<std::string>& labels, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& trainingSamples){

    //clean output vectors
    labels = std::vector<std::string>(0);
    trainingSamples = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>(0);

    std::string mainDirPath = ros::package::getPath("object_identifier") + "/clouds/";
    std::vector<std::string> samplePaths;

    //read library file
    std::ifstream samplesLibrary;
    std::string libraryFilePath = mainDirPath + "samples.txt";
    ROS_INFO("trying to open file %s", libraryFilePath.c_str());
    samplesLibrary.open(libraryFilePath.c_str());

    if(samplesLibrary.is_open()){
        int nOfLabels;
        samplesLibrary >> nOfLabels;

        std::vector<std::string> labelNames(nOfLabels);

        for(size_t i = 0; i < nOfLabels && !samplesLibrary.eof(); i++){
            int labelIndex;
            std::string labelName;
            samplesLibrary >> labelIndex;
            samplesLibrary >> labelName;

            labelNames[labelIndex] = labelName;
        }

        while(!samplesLibrary.eof()){
            int labelIndex = -1;
            std::string path;
            samplesLibrary >> labelIndex;
            samplesLibrary >> path;

            if(labelIndex == -1){
                continue;
            }

            labels.push_back(labelNames[labelIndex]);
            samplePaths.push_back(path);
        }
    }
    else{
        ROS_ERROR("unable to read training file library");
        return false;
    }

    //print read paths for debugging
    for(size_t i = 0; i < samplePaths.size(); i++){
        ROS_INFO("%s: %s", labels[i].c_str(), samplePaths[i].c_str());
    }

    //read sample clouds
    trainingSamples = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>(samplePaths.size());
    for(size_t i = 0; i < samplePaths.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string completeFilePath = mainDirPath + samplePaths[i];
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (completeFilePath, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s\n", samplePaths[i].c_str());
        }
        trainingSamples[i] = cloud;
    }
    return true;
}

std::string ObjectIdentifier::identifySingleObject(const pcl::PointCloud<POINTTYPE>::Ptr& object){

    std::string shape;

    //convert to have same pointtype (error otherwise...)
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzObject(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*object, *xyzObject);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(10);
    icp.setEuclideanFitnessEpsilon(0.0000001);
    icp.setInputTarget(xyzObject);

    double smallestError = 100;
    int smallestErrorIndex = -1;

    for(size_t i = 0; i < trainingSamples.size(); i++){
        icp.setInputSource(trainingSamples[i]);

        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final);
        double error = icp.getFitnessScore();

        if(error < smallestError){
            smallestError = error;
            smallestErrorIndex = i;
        }
    }
    if(smallestError > 0.000006){
        shape = "UNKNOWN";
    }
    else{
        shape = trainingSampleLabels[smallestErrorIndex];
    }
    return shape;
}

}//namespace primesense_pkgs

