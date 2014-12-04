#ifndef PRIMESENSE_PKGS_IMAGE_BUFFER_H
#define PRIMESENSE_PKGS_IMAGE_BUFFER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_buffer/EvidenceCommand.h>

#include <list>

#define IMAGE_BUFFER_TIME_S 5

namespace primesense_pkgs{

class ImageBuffer{

public:
    ImageBuffer();

private:
    ros::Subscriber imageSub;
    ros::Subscriber evidenceCommandSub;
    ros::Publisher evidencePub;

    sensor_msgs::Image lastImageMsg;
    std::list<sensor_msgs::Image> bufferList;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void evidenceCommandCallback(const image_buffer::EvidenceCommand::ConstPtr &msg);
    double computeTimeDiff(const ros::Time &t1, const ros::Time &t2);
};
}//namespace primesense_pkgs

#endif //PRIMESENSE_PKGS_IMAGE_BUFFER_H
