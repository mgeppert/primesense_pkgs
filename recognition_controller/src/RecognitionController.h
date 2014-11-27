#ifndef PRIMESENSE_PKGS_RECOGNITION_CONTROLLER_H
#define PRIMESENSE_PKGS_RECOGNITION_CONTROLLER_H

#include <ros/ros.h>

#include <object_finder/Positions.h>
#include <object_identifier/Objects.h>
#include <geometry_msgs/Twist.h>
#include <ocv_msgs/ocv.h>

#include <list>

#define OBSERVED_TIME_INTERVAL_SEC 5
#define ALLOWED_TIME_DIFFERENCE_SEC 0.05
#define ALLOWED_POSITION_DIFFERENCE_M 0.05
#define MIN_POINTCLOUD_DISTANCE_M 0.3 //TO CHECK
#define MAX_POINTCLOUD_DISTANCE_M 0.6 //TO CHECK
#define MIN_VOTES_FOR_OBJECT 2
#define MIN_VOTES_FOR_POSITION 5

namespace primesense_pkgs{

class RecognitionController{

public:
    RecognitionController();

private:    
    struct coordinates2D{
        double x;
        double y;
    };

    struct object_vote{
        std::string color;
        std::string shape;
        size_t votes;
    };

    struct unknown_object{
        coordinates2D coordinates; //average position
        size_t nPos; //number of positions for average

        std::list<object_vote> votes;
    };

    struct known_object{
        coordinates2D coordinates; //global position
        std::string color;
        std::string shape;
    };

    struct pose{
        ros::Time stamp;
        geometry_msgs::Twist pose;
    };

    std::list<unknown_object> objects_to_identify;
    std::list<known_object> identified_objects;
    std::list<pose> lastPositions;

    ros::Subscriber pcPosSub; //positions from pointcloud
    ros::Subscriber pcObjSub; //objects from pointcloud
    ros::Subscriber ocvSub; //objects + positions from openCV
    ros::Subscriber poseSub; //pose of robot in global space
    ros::Publisher pcPosPub; //positions (in robot space) to identify
    ros::Publisher identObjPub; //identified objects in global space
    ros::Publisher espeakPub; //tell identified object's name
    ros::Publisher evidenceCommandPub; //command to image buffer to send evidence image

    void pcPosCallback(const object_finder::Positions::ConstPtr &msg);
    void pcObjCallback(const object_identifier::Objects::ConstPtr &msg);
    void ocvCallback(const ocv_msgs::ocv::ConstPtr &msg); //TODO: add argument
    void globalPoseCallback(const geometry_msgs::Twist::ConstPtr &msg);

    bool positionTooOld(const RecognitionController::pose &pos);
    double computeTimeDiff(const ros::Time &t1, const ros::Time &t2);
    double computePositionDiff(const RecognitionController::coordinates2D & pos1, const RecognitionController::coordinates2D &pos2);

    bool addObjectPosition(ros::Time timestamp, coordinates2D coordinates, double angle);
    void addObjectVote(ros::Time timestamp, coordinates2D coordinates, std::string color, std::string shape, bool voteForPosition = false);
    void voteForObject(std::list<unknown_object>::iterator objectIt, std::string color, std::string shape, ros::Time timestamp);
    void decideOnObject(std::list<unknown_object>::iterator objectIt, std::string color, std::string shape, ros::Time timestamp);
    bool findTimePose(ros::Time timestamp, geometry_msgs::Twist &globalPose);
    coordinates2D computeGlobalPosition(geometry_msgs::Twist globalRobotPose, coordinates2D relativeObjectPosition);
    bool timeMatch(const ros::Time &t1, const ros::Time &t2);
    void updateObjectPosition(RecognitionController::unknown_object &object, RecognitionController::coordinates2D newPosition);
    void sendPositionToIdentifyObject(ros::Time timestamp, RecognitionController::coordinates2D relativeObjectPosition, double objectAngle);

};

} //namespace primesense_pkgs

#endif //PRIMESENSE_PKGS_RECOGNITION_CONTROLLER_H
