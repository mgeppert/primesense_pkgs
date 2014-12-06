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

void ImageBuffer::imageCallback(const sensor_msgs::Image::ConstPtr &msg){
    lastImageMsg = *msg;
    // ROS_INFO("New Image");
    // ROS_INFO("Time diff to HEAD = %f",computeTimeDiff(bufferList.front().header.stamp, lastImageMsg.header.stamp));
     if(bufferList.size()==0 || (bufferList.size() > 0 && 0.0 < computeTimeDiff(bufferList.front().header.stamp, lastImageMsg.header.stamp))){
        //add new image to buffer
        bufferList.push_front(lastImageMsg);
        // ROS_INFO("Size of bufferList= %lu", bufferList.size());
        // ROS_INFO("Image TimeStamp = %d", lastImageMsg.header.stamp.sec );
        //remove too old messages from buffer
        while(bufferList.size() > 0 && computeTimeDiff(bufferList.back().header.stamp, ros::Time::now()) > IMAGE_BUFFER_TIME_S){
            bufferList.pop_back();
        }
        // ROS_INFO("Size of bufferList= %lu", bufferList.size());
    }
    return;
}

void ImageBuffer::evidenceCommandCallback(const image_buffer::EvidenceCommand::ConstPtr &msg){
    ras_msgs::RAS_Evidence evidence;
    evidence.stamp = msg->header.stamp;
    evidence.group_number = 2;
    evidence.object_id = msg->object_name.data;

    //find best picture
    if (bufferList.empty()) {
    	// ROS_INFO("BL Empty");
			evidencePub.publish(evidence);
			return;
		}
    
    double smallestDiff = 100.0; //10000000000000.0;
    std::list<sensor_msgs::Image>::iterator bestIt;
    // ROS_INFO("2 Size of bufferList= %lu", bufferList.size());
    bestIt = bufferList.end();
    for(std::list<sensor_msgs::Image>::iterator it = bufferList.begin(); it != bufferList.end(); ++it){
        double currDiff = computeTimeDiff(it->header.stamp, msg->header.stamp);
        if(currDiff < smallestDiff){
            smallestDiff = currDiff;
            bestIt = it;
            // ROS_INFO("BestIt");
        }
        else {
        	//break;
        }
    }
    // ROS_INFO("Should Publish now");
    
    if(bestIt != bufferList.end()){
    	evidence.image_evidence = *bestIt;
    	// ROS_INFO("Publishing");
    	evidencePub.publish(evidence);
    }
    
    return;
}

double ImageBuffer::computeTimeDiff(const ros::Time &t1, const ros::Time &t2){
    int nSecDiff = abs(t1.nsec - t2.nsec);
    int secDiff = abs(t1.sec - t2.sec);
    // ROS_INFO("SEC t1=%d , t2=%d, DIFF=%d and absDIFF=%d", t1.sec, t2.sec,(t1.sec - t2.sec), secDiff);
    // ROS_INFO("nSEC nt1=%d , nt2=%d, nDIFF=%d and nabsDIFF=%d", t1.nsec, t2.nsec,(t1.nsec - t2.nsec), nSecDiff);
    double diff = (double) secDiff + (((double) nSecDiff) * (1e-9));
    // ROS_INFO("Total DIFF=%f", diff);

    return diff;
}
}
