#include "ImageBuffer.h"

#include <ras_msgs/RAS_Evidence.h>

namespace primesense_pkgs{

ImageBuffer::ImageBuffer(){
    ros::NodeHandle nh;

    imageSub = nh.subscribe("camera/rgb/image_raw", 1, &ImageBuffer::imageCallback, this);
    evidenceCommandSub = nh.subscribe("/recognition_controller/evidence_command", 1, &ImageBuffer::evidenceCommandCallback, this);
    evidencePub = nh.advertise<ras_msgs::RAS_Evidence>("/evidence", 1);

    return;
}

void ImageBuffer::buf(){
    if(bufferList.size() > 0 && 0.1 < computeTimeDiff(bufferList.front().header.stamp, lastImageMsg.header.stamp)){
        //add new image to buffer
        bufferList.push_front(lastImageMsg);

        //remove too old messages from buffer
        while(bufferList.size() > 0 && computeTimeDiff(bufferList.back().header.stamp, ros::Time::now()) > IMAGE_BUFFER_TIME_S){
            bufferList.pop_back();
        }
    }
    return;
}

void ImageBuffer::imageCallback(const sensor_msgs::Image::ConstPtr &msg){
    lastImageMsg = *msg;
    return;
}

void ImageBuffer::evidenceCommandCallback(const image_buffer::EvidenceCommand::ConstPtr &msg){
    ras_msgs::RAS_Evidence evidence;
    evidence.stamp = msg->header.stamp;
    evidence.group_number = 2;
    evidence.object_id = msg->object_name.data;

    //find best picture
    double smallestDiff = 100;
    std::list<sensor_msgs::Image>::iterator bestIt;
    for(std::list<sensor_msgs::Image>::iterator it = bufferList.begin(), end = bufferList.end(); it != end; it++){
        double currDiff = computeTimeDiff(it->header.stamp, msg->header.stamp);
        if(currDiff < smallestDiff){
            smallestDiff = currDiff;
            bestIt = it;
        }
        //else break??
    }

    evidence.image_evidence = *bestIt;

    evidencePub.publish(evidence);
    return;
}

double ImageBuffer::computeTimeDiff(const ros::Time &t1, const ros::Time &t2){
    int nSecDiff = std::abs(t1.nsec - t2.nsec);
    int secDiff = t1.sec - t2.sec;
    double diff = (double) secDiff + (((double) nSecDiff) / 1e-9);

    return diff;
}
}
