#include "ObjectIdentifier.h"

#include <object_identifier/Objects.h>

#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

#include <fstream>
#include <cmath>

namespace primesense_pkgs{

ObjectIdentifier::ObjectIdentifier(){

    ros::NodeHandle nh;
    cloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/cloud_preparation/prepared_cloud", 1);
    cloudSub->registerCallback(&ObjectIdentifier::cloudCallback, this);
    positionSub = new message_filters::Subscriber<object_finder::Positions>(nh, "/object_identifier/positions_in", 1);
    positionSub->registerCallback(&ObjectIdentifier::positionCallback, this);
    synchronizer = new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, object_finder::Positions>(*cloudSub, *positionSub, 1000);
    synchronizer->registerCallback(&ObjectIdentifier::cloudPositionCallback, this);

    objectPub = nh.advertise<object_identifier::Objects>("/object_identifier/objects", 1);
    debugPub = nh.advertise<sensor_msgs::PointCloud2>("/object_identifier/debug", 1);
    speakerPub = nh.advertise<std_msgs::String>("/espeak/string", 10);
    currentObjectsTimestamp = ros::Time();

//    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);

    if(!loadTrainingData(trainingSampleLabels, trainingSamples)){
        ROS_ERROR("unable to read training data");
    }

    return;
}

void ObjectIdentifier::identifyObjects(std::vector<pcl::PointXYZ> objectPositions, std::vector<double> objectRotations, const pcl::PointCloud<POINTTYPE>::Ptr &inputCloud){

    size_t nOfOrigPos = objectPositions.size();
    removeDuplicatePositions(objectPositions, objectRotations);
    size_t nOfNewPos = objectPositions.size();
    ROS_INFO("romeved %lu duplicate positions (now %lu instead of %lu)", nOfOrigPos - nOfNewPos, nOfNewPos, nOfOrigPos);
    std::vector<pcl::PointCloud<POINTTYPE>::Ptr> objectClouds = extractObjectClouds(objectPositions, objectRotations, inputCloud);

    std::vector<std::string> colors;
    std::vector<std::string> shapes;
    std::vector<pcl::PointXYZ> positions;

    for(size_t i = 0; i < objectClouds.size() && ros::ok(); i++){

        if(objectClouds[i]->points.size() < 200){
            ROS_ERROR("Not enough points in object cloud %lu", i);
            continue;
        }

        std::string color = getObjectColor(objectClouds[i]);

        if(color.compare("white") == 0){
            ROS_ERROR("object color is white");
            continue;
        }

        std::string shape = identifySingleObject(objectClouds[i], color);

        colors.push_back(color);
        shapes.push_back(shape);
        positions.push_back(objectPositions[i]);

//        std_msgs::String stringMsg;
//        stringMsg.data = "I see a " + color + " " + shape;
//        speakerPub.publish(stringMsg);

        ROS_INFO("object %lu: %s %s at position (%f, %f)", i, color.c_str(), shape.c_str(), objectPositions[i].x, objectPositions[i].z);
    }

    publishFoundObjects(colors, shapes, positions);
    return;
}

void ObjectIdentifier::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    //just for easier debugging
//    ROS_INFO("received cloud (ts: %d.%d)", msg->header.stamp.sec, msg->header.stamp.nsec);
    return;
}

void ObjectIdentifier::positionCallback(const object_finder::Positions::ConstPtr &msg){
    //just for easier debugging
//    ROS_INFO("received positions (ts: %d.%d)", msg->header.stamp.sec, msg->header.stamp.nsec);
    return;
}

void ObjectIdentifier::cloudPositionCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg, const object_finder::Positions::ConstPtr &posMsg){
    ROS_INFO("received cloud and positions");

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud(new pcl::PointCloud<POINTTYPE>);
    pcl::fromROSMsg(*cloudMsg, *inputCloud);

    currentObjectsTimestamp = cloudMsg->header.stamp;

    std::vector<pcl::PointXYZ> objectPositions = std::vector<pcl::PointXYZ>(posMsg->object_positions.size());
    std::vector<double> objectRotations = std::vector<double>(posMsg->object_positions.size());

    for(size_t i = 0; i < posMsg->object_positions.size(); i++){
        objectPositions[i].x = posMsg->object_positions[i].x;
        objectPositions[i].y = posMsg->object_positions[i].y;
        objectPositions[i].z = posMsg->object_positions[i].z;

        objectRotations[i] = posMsg->object_angles[i];
    }

    identifyObjects(objectPositions, objectRotations, inputCloud);

    return;
}

std::vector<pcl::PointCloud<POINTTYPE>::Ptr> ObjectIdentifier::extractObjectClouds(const std::vector<pcl::PointXYZ> &objectPositions, const std::vector<double> &objectRotations, const pcl::PointCloud<POINTTYPE>::Ptr &inputCloud){

    pcl::CropBox<POINTTYPE> cb;
    cb.setInputCloud(inputCloud);

    std::vector<pcl::PointCloud<POINTTYPE>::Ptr> objectClouds = std::vector<pcl::PointCloud<POINTTYPE>::Ptr>(objectPositions.size(), pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>));

    for(size_t i = 0; i < objectPositions.size(); i++){
        pcl::PointXYZ op = objectPositions[i];
        cb.setMin(Eigen::Vector4f(op.x - 0.05, 0.0, op.z - 0.05, 1.0));
        cb.setMax(Eigen::Vector4f(op.x + 0.05, 0.05, op.z + 0.05, 1.0));

        pcl::PointCloud<POINTTYPE>::Ptr extractedCloud(new pcl::PointCloud<POINTTYPE>);
//        cb.filter(*objectClouds[i]);
        cb.filter(*extractedCloud);

        //move object to origin
        pcl::PointCloud<POINTTYPE>::Ptr originCloud(new pcl::PointCloud<POINTTYPE>);
        Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
        transform_translation.translation() << -objectPositions[i].x, 0.0, -objectPositions[i].z;
        pcl::transformPointCloud (*extractedCloud, *originCloud, transform_translation);

        //rotate
        pcl::PointCloud<POINTTYPE>::Ptr alignedCloud(new pcl::PointCloud<POINTTYPE>);
        Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
        transform_rotation.rotate(Eigen::AngleAxisf(-objectRotations[i], Eigen::Vector3f::UnitY()));
        pcl::transformPointCloud(*originCloud, *alignedCloud, transform_rotation);

        objectClouds[i] = alignedCloud;
    }


    if(objectClouds.size() > 0){
        ROS_INFO("sending cloud of first object");
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*objectClouds[0], msg);
        debugPub.publish(msg);
    }
    return objectClouds;
}

bool ObjectIdentifier::loadTrainingData(std::vector<std::string>& labelNames, std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> >& trainingSamples){

    //clean output vectors
    labelNames = std::vector<std::string>(0);
    trainingSamples = std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> >(0);

    std::string mainDirPath = ros::package::getPath("object_identifier") + "/clouds/";
    std::vector<std::string> samplePaths;

    //read library file
    std::ifstream samplesLibrary;
    std::string libraryFilePath = mainDirPath + "samples.txt";
    ROS_INFO("trying to open file %s", libraryFilePath.c_str());
    samplesLibrary.open(libraryFilePath.c_str());

    std::vector<int> labels(0);

    if(samplesLibrary.is_open()){
        int nOfLabels;
        samplesLibrary >> nOfLabels;

        labelNames = std::vector<std::string>(nOfLabels);

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

            labels.push_back(labelIndex);
            samplePaths.push_back(path);
        }
    }
    else{
        ROS_ERROR("unable to read training file library");
        return false;
    }

    //print read paths for debugging
    for(size_t i = 0; i < samplePaths.size(); i++){
        ROS_INFO("%s: %s", labelNames[labels[i]].c_str(), samplePaths[i].c_str());
    }

    //read sample clouds
    trainingSamples = std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> >(labels.size());
    for(size_t i = 0; i < samplePaths.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string completeFilePath = mainDirPath + samplePaths[i];
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (completeFilePath, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s\n", samplePaths[i].c_str());
        }
        trainingSamples[labels[i]].push_back(cloud);
    }
    return true;
}

std::string ObjectIdentifier::identifySingleObject(const pcl::PointCloud<POINTTYPE>::Ptr& object, std::string color){

    if(color.compare("Orange") == 0){
        return "patrick";
    }
    else if(color.compare("Purple") == 0){
        return "cross";
    }

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

//    for(size_t i = 0; i < trainingSamples.size() && ros::ok(); i++){
//        icp.setInputSource(trainingSamples[i]);

//        pcl::PointCloud<pcl::PointXYZ> final;
//        icp.align(final);
//        double error = icp.getFitnessScore();

//        if(error < smallestError){
//            smallestError = error;
//            smallestErrorIndex = i;
//        }
//    }

    std::vector<int> possibleShapeIndices = getPossibleShapeIndices(color, trainingSampleLabels);
    for(size_t i = 0; i < possibleShapeIndices.size(); i++){
        for(size_t j = 0; j < trainingSamples[possibleShapeIndices[i]].size(); j++){
            icp.setInputSource(trainingSamples[possibleShapeIndices[i]][j]);

            pcl::PointCloud<pcl::PointXYZ> final;
            icp.align(final);
            double error = icp.getFitnessScore();

            if(error < smallestError){
                smallestError = error;
                smallestErrorIndex = i;
            }
        }
    }
    if(smallestError > 0.0006){
        shape = "UNKNOWN";
    }
    else{
        shape = trainingSampleLabels[smallestErrorIndex];
    }
    return shape;
}

void ObjectIdentifier::removeDuplicatePositions(std::vector<pcl::PointXYZ> &objectPositions, std::vector<double> &objectRotations){

    if(objectPositions.size() > 0){

        std::vector<pcl::PointXYZ> newPositions;
        std::vector<double> newRotations;

        newPositions.push_back(objectPositions[0]);

        for(size_t i = 1; i < objectPositions.size(); i++){
            bool tooClose = false;
            for(size_t j = 0; j < newPositions.size(); j++){
                if(std::sqrt(std::pow(objectPositions[i].x - newPositions[j].x, 2) + std::pow(objectPositions[i].z - newPositions[j].z, 2)) < 0.05){
                    tooClose= true;
                    break;
                }
            }
            if(!tooClose){
                newPositions.push_back(objectPositions[i]);
                newRotations.push_back(objectRotations[i]);
            }
        }
        objectPositions = newPositions;
        objectRotations = newRotations;
    }
}

std::string ObjectIdentifier::getObjectColor(const pcl::PointCloud<POINTTYPE>::Ptr &object){

    //get mean rgb color
    unsigned int r = 0;
    unsigned int g = 0;
    unsigned int b = 0;

    for(size_t i = 0; i < object->points.size(); i++){
        r += object->points[i].r;
        g += object->points[i].g;
        b += object->points[i].b;
    }

    r /= object->points.size();
    g /= object->points.size();
    b /= object->points.size();

    ROS_INFO("mean rgb color: %u, %u, %u", r, g, b);

    //convert to hsv
    double R = (double) r / 255.0d;
    double G = (double) g / 255.0d;
    double B = (double) b / 255.0d;

    double cMax = std::max(std::max(R, G), B);
    double cMin = std::min(std::min(R, G), B);
    double delta = cMax - cMin;

    double H;
    if(cMax == R){
        H = ((G - B)/ delta) * 60;
    }
    else if(cMax == G){
        H = ((B - R) / delta + 2) * 60;
    }
    else{ //cMax == B
        H = ((R - G) / delta + 4) * 60;
    }

    if(H < 0){
        H += 360.0d;
    }

    double S;
    if(cMax == 0){
        S = 0;
    }
    else{
        S = delta / cMax;
    }

    double V = cMax;

    ROS_INFO("mean hsv color: %f, %f, %f", H, S, V);

    std::string color = classifyColor(H, S, V);

    return color;
}

std::string ObjectIdentifier::classifyColor(double h, double s, double v){

    //ensure correct hsv values
    if(h < 0 || h > 360 || s < 0 || s > 1 || v < 0 || v > 1){
        ROS_ERROR("invalid hsv values in classifyColor");
        return "invalid hsv color";
    }

    std::string color;
    color = "Unknown Color";

    if(s < 0.1){
        color = "White";
    }
    else if(h <= 20){
        if(s >= 2.0d / 3.0d && v >= 2.0d / 3.0d){
            color = "Orange";
        }
    }
    else if(h >= 60 && h <= 178){
        if(s >= 0.313 && v >= 0.235){
            color = "Green";
        }
    }
    else if(h >= 180 && h <= 238){
        if(s >= 0.114 && v >= 0.235){
            color = "Blue";
        }
    }
    else if(h >= 240 && h <= 320){
        if(s >= 0.353 && v >= 0.313){
            color = "Purple";
        }
    }
    else if(h >= 322 && h <= 358){
        if(s >= 0.196 && v >= 0.196){
            color = "Red";
        }
    }
    return color;
}

void ObjectIdentifier::publishFoundObjects(const std::vector<std::string>& colors, const std::vector<std::string>& shapes, const std::vector<pcl::PointXYZ>& positions){

    object_identifier::Objects objectsMsg;
    objectsMsg.header = std_msgs::Header();
    objectsMsg.header.stamp = currentObjectsTimestamp;

    for(size_t i = 0; i < colors.size(); i++){

        std_msgs::String shapeString;
        shapeString.data = shapes[i].compare("UNKNOWN") == 0 ? "Object" : shapes[i];
        objectsMsg.shapes.push_back(shapeString);

        std_msgs::String colorString;
        colorString.data = colors[i].compare("Unknown Color") == 0 ? "" : colors[i];
        objectsMsg.colors.push_back(colorString);

        geometry_msgs::Point position;
        position.x = positions[i].x;
        position.y = positions[i].y;
        position.z = positions[i].z;

        objectsMsg.positions.push_back(position);
    }

    objectPub.publish(objectsMsg);

    return;
}

std::vector<int> ObjectIdentifier::getPossibleShapeIndices(std::string color, const std::vector<std::string> &shapes){

    std::vector<std::string> possibleShapes;
    std::vector<int> possibleShapeIndices;

    if(color.compare("Green") == 0){
        possibleShapes.push_back("Cube");
        possibleShapes.push_back("Cylinder");
    }
    else if(color.compare("Red") == 0){
        possibleShapes.push_back("Ball");
        possibleShapes.push_back("Cube");
    }
    else if(color.compare("Yellow") == 0){
        possibleShapes.push_back("Ball");
        possibleShapes.push_back("Cube");
    }
    else if(color.compare("Blue") == 0){
        possibleShapes.push_back("Cube");
        possibleShapes.push_back("Triangle");
    }
    else {
        //test all shapes
        possibleShapes.push_back("Ball");
        possibleShapes.push_back("Cube");
        possibleShapes.push_back("Cylinder");
        possibleShapes.push_back("Triangle");
    }

    for(size_t i = 0; i < possibleShapes.size(); i++){
        for(size_t j = 0; j < shapes.size(); j++){
            if(possibleShapes[i].compare(shapes[j]) == 0){
                possibleShapeIndices.push_back(j);
            }
        }
    }
    return possibleShapeIndices;
}

}//namespace primesense_pkgs
