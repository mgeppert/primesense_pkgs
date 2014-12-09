#include "ObjectFinder.h"

#include <object_finder/Positions.h>
#include <object_finder/WallPoints.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>

#include <algorithm>

namespace primesense_pkgs{

ObjectFinder::ObjectFinder(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/camera/depth_registered/points", 1, &ObjectFinder::cloudCallback, this);
    objectsPub = nh.advertise<object_finder::Positions>("object_finder/positions", 1);
    upperProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("/object_finder/upper_projection", 1);
    lowerProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("/object_finder/lower_projection", 1);
    differencesPub = nh.advertise<sensor_msgs::PointCloud2>("/object_finder/differences", 1);
    markerPub = nh.advertise<visualization_msgs::Marker>("/object_finder/marker", 1);
    wallPointsPub = nh.advertise<object_finder::WallPoints>("/object_finder/wallpoints", 1);

    lowerBox = pcl::CropBox<POINTTYPE>();
    lowerBox.setMin(Eigen::Vector4f(-10.0, 0.01, 0.0, 1.0));
    lowerBox.setMax(Eigen::Vector4f(10.0, 0.06, 1.0, 1.0));

    upperBox = pcl::CropBox<POINTTYPE>();
    upperBox.setMin(Eigen::Vector4f(-10.0, 0.06, 0.0, 1.0));
    upperBox.setMax(Eigen::Vector4f(10.0, 0.25, 1.0, 1.0));

    smallBox = pcl::CropBox<POINTTYPE>();
    smallBox.setMin(Eigen::Vector4f(-0.4, 0.0, 0.1, 1.0));
    smallBox.setMax(Eigen::Vector4f(0.4, 0.25, 1.0, 1.0));

    triangleBox = pcl::CropBox<POINTTYPE>();
    triangleBox.setMin(Eigen::Vector4f(-0.4, 0.0, 0.1, 1.0));
    triangleBox.setMax(Eigen::Vector4f(0.4, 0.25, 1.0, 1.0));
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform(0,2) = -0.4d/1.0d;
    transform(0,3) = 0.4;
    triangleBox.setTransform(transform);

//    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);
//    currentCloudTimeStamp = ros::Time::now();

    return;
}

void ObjectFinder::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
//    ROS_INFO("received cloud");

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud(new pcl::PointCloud<POINTTYPE>);
    pcl::fromROSMsg(*msg, *inputCloud);
    ros::Time currentCloudTimeStamp = msg->header.stamp;

    pcl::PointCloud<POINTTYPE>::Ptr dsCloud = downSample(inputCloud);
    pcl::PointCloud<POINTTYPE>::Ptr adaptedCloud = adaptViewPoint(dsCloud);
    findObjects(adaptedCloud, currentCloudTimeStamp);
    return;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::adaptViewPoint(const pcl::PointCloud<POINTTYPE>::Ptr &cloud){

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    //mirror cloud at x-z plane to get positive y values
    transform(1, 1) = -1;

    double height = 0.0;
    ros::param::getCached("/calibration/height", height);
    transform.translation() << 0.0, height, 0.0;
    double theta_x = 0.0;
    ros::param::getCached("/calibration/x_angle", theta_x);

    // rotate tetha radians arround X axis
    transform.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));

    ROS_INFO("rotate by %f rads around x-axis", theta_x);

    // Executing the transformation
    pcl::PointCloud<POINTTYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINTTYPE>());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    return transformed_cloud;
}


void ObjectFinder::findObjects(const pcl::PointCloud<POINTTYPE>::Ptr &inputCloud, ros::Time currentCloudTimeStamp){

    pcl::PointCloud<POINTTYPE>::Ptr upperCloud = cropUpperBox(inputCloud);
    pcl::PointCloud<POINTTYPE>::Ptr lowerCloud = cropLowerBox(inputCloud);

//    ROS_INFO("#points: upper: %lu, lower: %lu", upperCloud->points.size(), lowerCloud->points.size());

    sendWallPoints(upperCloud, currentCloudTimeStamp);

    pcl::PointCloud<POINTTYPE>::Ptr upperProjection = projectToZeroPlane(upperCloud);
    pcl::PointCloud<POINTTYPE>::Ptr lowerProjection = projectToZeroPlane(lowerCloud);

    sensor_msgs::PointCloud2 upperProjectionMsg;
    pcl::toROSMsg(*upperProjection, upperProjectionMsg);
    upperProjectionPub.publish(upperProjectionMsg);

    sensor_msgs::PointCloud2 lowerProjectionMsg;
    pcl::toROSMsg(*lowerProjection, lowerProjectionMsg);
    lowerProjectionPub.publish(lowerProjectionMsg);

    upperProjection = cropTriangleBox(upperProjection);
    lowerProjection = cropTriangleBox(lowerProjection);

//    ROS_INFO("#points: upperProjection: %lu, lowerProjection: %lu", upperProjection->points.size(), lowerProjection->points.size());

    pcl::PointCloud<POINTTYPE>::Ptr differenceCloud = getDifference(upperProjection, lowerProjection);
//    ROS_INFO("#points in differenceCloud: %lu", differenceCloud->points.size());

    sensor_msgs::PointCloud2 differenceCloudMsg;
    pcl::toROSMsg(*differenceCloud, differenceCloudMsg);
    differencesPub.publish(differenceCloudMsg);

    std::vector<ObjectFinder::objectPose> objectPoses = getObjectPoses(differenceCloud);

    std::sort(objectPoses.begin(), objectPoses.end(), ObjectFinder::positionCompare);

    object_finder::Positions object_pos;
    object_pos.header = std_msgs::Header();
    object_pos.header.stamp = currentCloudTimeStamp;

    for(size_t i = 0; i < objectPoses.size(); i++){
        geometry_msgs::Point point;
        point.x = objectPoses[i].position.x;
        point.y = objectPoses[i].position.y;
        point.z = objectPoses[i].position.z;

        object_pos.object_positions.push_back(point);
        object_pos.object_angles.push_back(objectPoses[i].angle);

        sendMarker(objectPoses[i].position, i, currentCloudTimeStamp);
    }

    objectsPub.publish(object_pos);

    return;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::downSample(const pcl::PointCloud<POINTTYPE>::Ptr &pc){

    pcl::ApproximateVoxelGrid<POINTTYPE> grid;
    grid.setLeafSize(0.004, 0.01, 0.004);
    grid.setInputCloud(pc);

    pcl::PointCloud<POINTTYPE>::Ptr dsCloud(new pcl::PointCloud<POINTTYPE>);
    grid.filter(*dsCloud);

    return dsCloud;
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

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::cropTriangleBox(const pcl::PointCloud<POINTTYPE>::Ptr& pc){

    smallBox.setInputCloud(pc);
    pcl::PointCloud<POINTTYPE>::Ptr smallCloud(new pcl::PointCloud<POINTTYPE>);
    smallBox.filter(*smallCloud);

    triangleBox.setInputCloud(smallCloud);

    pcl::PointCloud<POINTTYPE>::Ptr triangleCloud(new pcl::PointCloud<POINTTYPE>);
    triangleBox.filter(*triangleCloud);
    return triangleCloud;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::projectToZeroPlane(pcl::PointCloud<POINTTYPE>::Ptr pc){

    pcl::PointCloud<POINTTYPE>::Ptr projectedCloud = pc->makeShared();

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 1;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<POINTTYPE> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (pc);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projectedCloud);

    return projectedCloud;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::getDifference(const pcl::PointCloud<POINTTYPE>::Ptr &upc, const pcl::PointCloud<POINTTYPE>::Ptr &lpc){

    pcl::PointCloud<POINTTYPE>::Ptr differenceCloud(new pcl::PointCloud<POINTTYPE>);

    pcl::SegmentDifferences<POINTTYPE> segmenter;
    pcl::search::KdTree<POINTTYPE>::Ptr kdtree(new pcl::search::KdTree<POINTTYPE>);

    segmenter.setSearchMethod(kdtree);
    segmenter.setDistanceThreshold(0.005);
    segmenter.setTargetCloud(upc);
    segmenter.setInputCloud(lpc);
    segmenter.segment(*differenceCloud);

    return differenceCloud;
}

std::vector<ObjectFinder::objectPose> ObjectFinder::getObjectPoses(const pcl::PointCloud<POINTTYPE>::Ptr &pc){

    pcl::search::KdTree<POINTTYPE>::Ptr kdTree (new pcl::search::KdTree<POINTTYPE>);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<POINTTYPE> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (75); //might have to be adjusted since the resolution of the voxel grid changed
    ec.setMaxClusterSize (5000);
    ec.setSearchMethod (kdTree);
    ec.setInputCloud (pc);
    ec.extract(cluster_indices);

    std::vector<ObjectFinder::objectPose> objectPoses(0);

//    ROS_INFO("%lu clusters detected", cluster_indices.size());

    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), end = cluster_indices.end(); it != end; it++){

//        ROS_INFO("size of original cloud: %lu", pc->points.size());
//        ROS_INFO("indices: %lu", it->indices.size());
        //extract object from original cloud
        pcl::PointCloud<POINTTYPE>::Ptr objectCloud(new pcl::PointCloud<POINTTYPE>);

        pcl::ExtractIndices<POINTTYPE> extract;
        extract.setInputCloud(pc);
        pcl::PointIndicesPtr inds(new pcl::PointIndices);
        inds->indices = it->indices;
        extract.setIndices(inds);
        extract.filter(*objectCloud);

//        ROS_INFO("size of objectCloud: %lu", objectCloud->points.size());

        Eigen::Vector4d massCentroid;
        pcl::compute3DCentroid(*objectCloud, massCentroid);
        pcl::PointXYZ centerOfMass;
        centerOfMass.x = massCentroid[0];
        centerOfMass.y = massCentroid[1];
        centerOfMass.z = massCentroid[2];

        //downsample again to get more uniform distribution of points
//        ROS_INFO("points before downsampling object: %lu", it->indices.size());
        pcl::ApproximateVoxelGrid<POINTTYPE> grid;
        grid.setLeafSize(0.005, 0.005, 0.005);
        grid.setInputCloud(objectCloud);
        pcl::PointCloud<POINTTYPE>::Ptr dsObject(new pcl::PointCloud<POINTTYPE>);
        grid.filter(*dsObject);
//        ROS_INFO("points after downsampling object: %lu", dsObject->points.size());

        Eigen::Vector4d midCentroid;
        pcl::compute3DCentroid(*dsObject, midCentroid);
        pcl::PointXYZ midPoint;
        midPoint.x = midCentroid[0];
        midPoint.y = midCentroid[1];
        midPoint.z = midCentroid[2];

        //compute angle
        double xVec = centerOfMass.x - midPoint.x;
        double zVec = centerOfMass.z - midPoint.z;

//        ROS_INFO("offset: %f, %f", xVec, zVec);

        //angle of the object in comparison to unit vector -z
        double angle = std::atan2(-zVec, xVec);

        ROS_INFO("object position: (%f, %f, %f); angle: %f", midPoint.x, midPoint.y, midPoint.z, angle);

        ObjectFinder::objectPose pose;
        pose.position = midPoint;
        pose.angle = angle;
        objectPoses.push_back(pose);
    }
    return objectPoses;
}

bool ObjectFinder::positionCompare(const ObjectFinder::objectPose& lhs, const ObjectFinder::objectPose& rhs){
    double lhsDist = std::sqrt(std::pow(lhs.position.x, 2) + std::pow(lhs.position.z, 2));
    double rhsDist = std::sqrt(std::pow(rhs.position.x, 2) + std::pow(rhs.position.z, 2));
    return lhsDist < rhsDist;
}

void ObjectFinder::sendMarker(pcl::PointXYZ point, int id, ros::Time timestamp){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = timestamp;
    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y + 0.02;
    marker.pose.position.z = point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    markerPub.publish(marker);
}

void ObjectFinder::sendWallPoints(const pcl::PointCloud<POINTTYPE>::Ptr &pc, ros::Time timestamp){
    //cut out box
    pcl::CropBox<POINTTYPE> box;
    box.setMin(Eigen::Vector4f(-0.15, 0, 0.1, 1.0));
    box.setMax(Eigen::Vector4f(0.15, 1, 0.75, 1.0));

    pcl::PointCloud<POINTTYPE>::Ptr boxCloud(new pcl::PointCloud<POINTTYPE>);
    box.setInputCloud(pc);
    box.filter(*boxCloud);

    ROS_INFO("points in 'wall cloud': %lu", boxCloud->points.size());
    //abort if too few points (likely only noise)
    if(boxCloud->points.size() < 20){
        ROS_INFO("not enough points - abort");
        return;
    }

    //sample down
    pcl::ApproximateVoxelGrid<POINTTYPE> grid;
    grid.setLeafSize(0.01, 100, 0.01);
    grid.setInputCloud(boxCloud);

    pcl::PointCloud<POINTTYPE>::Ptr dsCloud(new pcl::PointCloud<POINTTYPE>);
    grid.filter(*dsCloud);

//    sensor_msgs::PointCloud2 upperProjectionMsg;
//    pcl::toROSMsg(*dsCloud, upperProjectionMsg);
//    upperProjectionPub.publish(upperProjectionMsg);

    //create message
    object_finder::WallPoints msg;
    msg.header.stamp = timestamp;
    msg.points.resize(dsCloud->points.size());

    for(size_t i = 0; i < dsCloud->points.size(); i++){
        msg.points[i].x = dsCloud->points[i].x;
        msg.points[i].y = dsCloud->points[i].z;
        msg.points[i].z = 0;
    }

    wallPointsPub.publish(msg);
    return;
}

} //namespace primesense_pkgs
