#include "RecognitionController.h"

#include <cmath>

namespace primesense_pkgs{

RecognitionController::RecognitionController(){
    ros::NodeHandle nh;

    //TODO: add subscribers + publishers
    pcPosSub = nh.subscribe("/object_finder/positions", 1, &RecognitionController::pcPosCallback, this);
    pcObjSub = nh.subscribe("/object_identifier/objects", 1, &RecognitionController::pcObjCallback, this);
    ocvSub = nh.subscribe("/ocvrec/data", 1, &RecognitionController::ocvCallback, this);
    poseSub = nh.subscribe("/posOri/Twist", 1, &RecognitionController::globalPoseCallback, this);

    pcPosPub = nh.advertise<object_finder::Positions>("/object_identifier/positions_in", 1);
    //TODO: change this after debugging
    identObjPub = nh.advertise<std_msgs::String>("/recognition_controller/identified_objects", 1);
    espeakPub = nh.advertise<std_msgs::String>("/espeak/string", 1);

    objects_to_identify = std::list<RecognitionController::unknown_object>();
    identified_objects = std::list<RecognitionController::known_object>();
    lastPositions = std::list<RecognitionController::pose>();
    return;
}

void RecognitionController::pcPosCallback(const object_finder::Positions::ConstPtr &msg){

    for(size_t i = 0; i < msg->object_positions.size(); i++){
        coordinates2D coord;
        coord.x = msg->object_positions[i].x;
        coord.y = msg->object_positions[i].z;

        double angle = msg->object_angles[i];
        ros::Time timestamp = msg->header.stamp;

        //debug output
        ROS_INFO("received object at position (%f, %f) from object_finder", coord.x, coord.y);

        addObjectPosition(timestamp, coord, angle);
    }
    return;
}

void RecognitionController::pcObjCallback(const object_identifier::Objects::ConstPtr &msg){

    ros::Time timestamp = msg->header.stamp;

    for(size_t i = 0; i < msg->positions.size(); i++){
        coordinates2D coord;
        coord.x = msg->positions[i].x;
        coord.y = msg->positions[i].z;

        std::string color = msg->colors[i].data;
        std::string shape = msg->shapes[i].data;

        //debug output
        ROS_INFO("received %s %s at position (%f, %f) from object_identifier", color.c_str(), shape.c_str(), coord.x, coord.y);

        addObjectVote(timestamp, coord, color, shape, false);
    }
    return;
}

void RecognitionController::ocvCallback(const ocv_msgs::ocv::ConstPtr &msg){

    ros::Time timestamp = msg->header.stamp;

    coordinates2D coord;
    coord.x = msg->position.x;
    coord.y = msg->position.z;

    std::string color = msg->color.data;
    std::string shape = msg->shape.data;

    //debug output
    ROS_INFO("received %s %s at position (%f, %f) from openCV", color.c_str(), shape.c_str(), coord.x, coord.y);

    addObjectVote(timestamp, coord, color, shape, true);

    return;
}

void RecognitionController::globalPoseCallback(const geometry_msgs::Twist::ConstPtr &msg){
    ros::Time stamp = ros::Time::now();
    RecognitionController::pose pos;
    pos.stamp = stamp;
    pos.pose = *msg;

    lastPositions.push_front(pos);

    while(positionTooOld(lastPositions.back())){
        lastPositions.pop_back();
    }
    return;
}

bool RecognitionController::positionTooOld(const RecognitionController::pose &pos){
    ros::Time now = ros::Time::now();
    double diff = computeTimeDiff(now, pos.stamp);

    return diff > OBSERVED_TIME_INTERVAL_SEC;
}

double RecognitionController::computeTimeDiff(const ros::Time &t1, const ros::Time &t2){
    int nSecDiff = std::abs(t1.nsec - t2.nsec);
    int secDiff = t1.sec - t2.sec;
    double diff = (double) secDiff + (((double) nSecDiff) / 1e-9);

    return diff;
}

double RecognitionController::computePositionDiff(const RecognitionController::coordinates2D &pos1, const RecognitionController::coordinates2D &pos2){
    double distance = std::sqrt(std::pow(pos1.x - pos2.x, 2) + std::pow(pos1.y - pos2.y, 2));
    return distance;
}

bool RecognitionController::addObjectPosition(ros::Time timestamp, RecognitionController::coordinates2D coordinates, double angle){

    //compute global position
    geometry_msgs::Twist globalPose;
    if(!findTimePose(timestamp, globalPose)){
        ROS_ERROR("could not find suitable global robot pose");
        return false;
    }

    RecognitionController::coordinates2D globalObjectPosition = computeGlobalPosition(globalPose, coordinates);

    ROS_INFO("adding global position (%f, %f)", globalObjectPosition.x, globalObjectPosition.y);

    for(std::list<known_object>::const_iterator it = identified_objects.begin(), end = identified_objects.end(); it != end; it++){
        if(computePositionDiff(globalObjectPosition, it->coordinates) < ALLOWED_POSITION_DIFFERENCE_M){
            //seen object was already identified
            return false;
        }
    }

    bool objectExists = false;

    //check if position already exists + vote or add position
    for(std::list<RecognitionController::unknown_object>::iterator it = objects_to_identify.begin(), end = objects_to_identify.end(); it != end; it++){
        if(computePositionDiff(globalObjectPosition, it->coordinates) < ALLOWED_POSITION_DIFFERENCE_M){
            //object is the same

            //update object position
            updateObjectPosition(*it, globalObjectPosition);

            //try to recognize if position is sure
            if(it->nPos >= MIN_VOTES_FOR_POSITION){
                //check if object is in range

                //get global position of robot
                RecognitionController::coordinates2D globalRobotPosition;
                globalRobotPosition.x = globalPose.linear.x;
                globalRobotPosition.y = globalPose.linear.y;

                //compute distance
                double distance = computePositionDiff(globalRobotPosition, globalObjectPosition);

                if(distance >= MIN_POINTCLOUD_DISTANCE_M && distance <= MAX_POINTCLOUD_DISTANCE_M){
                    //send position to object_identifier
                    sendPositionToIdentifyObject(timestamp, coordinates, angle);
                }
            }
            objectExists = true;
            break;
        }
    }

    if(!objectExists){
        //add object to list
        RecognitionController::unknown_object newObj;
        newObj.coordinates = globalObjectPosition;
        newObj.nPos = 1;
        newObj.votes = std::list<RecognitionController::object_vote>();

        objects_to_identify.push_back(newObj);
    }
    return true;
}

void RecognitionController::addObjectVote(ros::Time timestamp, coordinates2D coordinates, std::string color, std::string shape, bool voteForPosition){

    ROS_INFO("vote for object %s %s at position (%f, %f)", color.c_str(), shape.c_str(), coordinates.x, coordinates.y);
    if(voteForPosition){//true when called after openCV message, otherwise we already have many votes for this position
        if(!addObjectPosition(timestamp, coordinates, 0)){
            //the object was already identified
            return;
        }
    }

    //do not vote if the respective object has also been selected somewhere else
    for(std::list<known_object>::const_iterator it = identified_objects.begin(), end = identified_objects.end(); it != end; it++){
        if(color.compare(it->color) == 0 && shape.compare(it->shape) == 0){
            return;
        }
    }

    //compute global position
    geometry_msgs::Twist globalPose;
    if(!findTimePose(timestamp, globalPose)){
        ROS_ERROR("could not find suitable global robot pose");
        return;
    }

    RecognitionController::coordinates2D globalObjectPosition = computeGlobalPosition(globalPose, coordinates);

    for(std::list<RecognitionController::unknown_object>::iterator it = objects_to_identify.begin(), end = objects_to_identify.end(); it != end; it++){
        if(computePositionDiff(globalObjectPosition, it->coordinates) < ALLOWED_POSITION_DIFFERENCE_M){
            //object is the same
            voteForObject(it, color, shape);
            return;
        }
    }
}

void RecognitionController::voteForObject(std::list<unknown_object>::iterator objectIt, std::string color, std::string shape){
    for(std::list<object_vote>::iterator it = objectIt->votes.begin(), end = objectIt->votes.end(); it != end; it++){
        if(color.compare(it->color) == 0 && shape.compare(it->shape) == 0){
            //found the same object
            it->votes++;

            if(it->votes >= MIN_VOTES_FOR_OBJECT){
                //high probability for this object -> decide on this
                decideOnObject(objectIt, color, shape);
                return;
            }
        }
    }

    //object not yet in the list
    object_vote newVote;
    newVote.color = color;
    newVote.shape = shape;
    newVote.votes = 1;
    objectIt->votes.push_back(newVote);
    return;
}

void RecognitionController::decideOnObject(std::list<unknown_object>::iterator objectIt, std::string color, std::string shape){
    //add to identified objects list
    known_object newKnownObect;
    newKnownObect.color = color;
    newKnownObect.shape = shape;
    newKnownObect.coordinates.x = objectIt->coordinates.x;
    newKnownObect.coordinates.y = objectIt->coordinates.y;

    //remove from unknown objects list
    objects_to_identify.erase(objectIt);

    ROS_INFO("identified object: %s %s at position (%f, %f)", color.c_str(), shape.c_str(), newKnownObect.coordinates.x, newKnownObect.coordinates.y);

    //TODO send message for map

    //send message to espeak
    std_msgs::String espeakMsg;
    espeakMsg.data = ("I see a " + color + " " + shape + ".").c_str();
    espeakPub.publish(espeakMsg);

    return;
}

bool RecognitionController::findTimePose(ros::Time timestamp, geometry_msgs::Twist &globalPose){
    for(std::list<RecognitionController::pose>::const_iterator it = lastPositions.begin(), end = lastPositions.end(); it != end; it++){
        if(timeMatch(timestamp, it->stamp)){
            globalPose = it->pose;
            return true;
        }
    }

    //------beginning of dummy code for debugging without pose messages
    geometry_msgs::Twist debugDummyMsg;
    debugDummyMsg.linear.x = 0;
    debugDummyMsg.linear.y = 0;
    debugDummyMsg.angular.z = 0;
    globalPose = debugDummyMsg;
    return true;
    //------end of dummy code for debugging

    return false;
}

RecognitionController::coordinates2D RecognitionController::computeGlobalPosition(geometry_msgs::Twist globalRobotPose, coordinates2D relativeObjectPosition){
    RecognitionController::coordinates2D globalObjectPosition;
    globalObjectPosition.x = globalRobotPose.linear.x + std::cos(globalRobotPose.angular.z)*relativeObjectPosition.y + std::sin(globalRobotPose.angular.z)*relativeObjectPosition.x;
    globalObjectPosition.y = globalRobotPose.linear.y + std::sin(globalRobotPose.angular.z)*relativeObjectPosition.y - std::cos(globalRobotPose.angular.z)*relativeObjectPosition.x;
    return globalObjectPosition;
}

bool RecognitionController::timeMatch(const ros::Time &t1, const ros::Time &t2){
    return computeTimeDiff(t1, t2) < ALLOWED_TIME_DIFFERENCE_SEC;
}

void RecognitionController::updateObjectPosition(RecognitionController::unknown_object &object, RecognitionController::coordinates2D newPosition){
    object.coordinates.x = ((object.coordinates.x * object.nPos) + newPosition.x) / (object.nPos + 1);
    object.coordinates.y = ((object.coordinates.y * object.nPos) + newPosition.y) / (object.nPos + 1);
    object.nPos++;
    return;
}

void RecognitionController::sendPositionToIdentifyObject(ros::Time timestamp, RecognitionController::coordinates2D relativeObjectPosition, double objectAngle){
    object_finder::Positions positionMsg;
    positionMsg.header = std_msgs::Header();
    positionMsg.header.stamp = timestamp;

    geometry_msgs::Point objectPosition;
    objectPosition.x = relativeObjectPosition.x;
    objectPosition.y = 0;
    objectPosition.z = relativeObjectPosition.y;

    positionMsg.object_positions.push_back(objectPosition);
    positionMsg.object_angles.push_back(objectAngle);

//    identObjPub.publish(positionMsg);
    ROS_INFO("ask for identification of object at (%f, %f)", relativeObjectPosition.x, relativeObjectPosition.y);
    pcPosPub.publish(positionMsg);
    return;
}

}//namespace primesense_pkgs
