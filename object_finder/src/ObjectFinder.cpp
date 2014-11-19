#include "ObjectFinder.h"

#include <object_finder/Positions.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>

#include <algorithm>

namespace primesense_pkgs{

ObjectFinder::ObjectFinder(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/cloud_preparation/prepared_cloud", 1, &ObjectFinder::cloudCallback, this);
    objectsPub = nh.advertise<object_finder::Positions>("object_finder/positions", 1);
    upperProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("/object_finder/upper_projection", 1);
    lowerProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("/object_finder/lower_projection", 1);
    differencesPub = nh.advertise<sensor_msgs::PointCloud2>("/object_finder/differences", 1);
    markerPub = nh.advertise<visualization_msgs::Marker>("/object_finder/marker", 1);

    lowerBox = pcl::CropBox<POINTTYPE>();
    lowerBox.setMin(Eigen::Vector4f(-10.0, 0.01, 0.0, 1.0));
    lowerBox.setMax(Eigen::Vector4f(10.0, 0.06, 2.5, 1.0));

    upperBox = pcl::CropBox<POINTTYPE>();
    upperBox.setMin(Eigen::Vector4f(-10.0, 0.06, 0.0, 1.0));
    upperBox.setMax(Eigen::Vector4f(10.0, 0.25, 2.5, 1.0));

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

    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);
    currentCloudTimeStamp = ros::Time::now();

    return;
}

void ObjectFinder::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
//    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *inputCloud);
    currentCloudTimeStamp = msg->header.stamp;
    return;
}

void ObjectFinder::findObjects(){

    pcl::PointCloud<POINTTYPE>::Ptr dsCloud = downSample(inputCloud);
    pcl::PointCloud<POINTTYPE>::Ptr upperCloud = cropUpperBox(dsCloud);
    pcl::PointCloud<POINTTYPE>::Ptr lowerCloud = cropLowerBox(dsCloud);

    ROS_INFO("#points: upper: %lu, lower: %lu", upperCloud->points.size(), lowerCloud->points.size());

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

//    ROS_INFO("lower projection points, before: %lu, after: %lu", lowerCloud->points.size(), lowerProjection->points.size());

    ROS_INFO("#points: upperProjection: %lu, lowerProjection: %lu", upperProjection->points.size(), lowerProjection->points.size());

//    std::vector<int> differenceIndices = getDifferenceIndices(upperProjection, lowerProjection);

//    pcl::PointCloud<POINTTYPE>::Ptr differenceCloud = getDifferenceCloud(lowerCloud, differenceIndices);

    pcl::PointCloud<POINTTYPE>::Ptr differenceCloud = getDifference(upperProjection, lowerProjection);
    ROS_INFO("#points in differenceCloud: %lu", differenceCloud->points.size());

    sensor_msgs::PointCloud2 differenceCloudMsg;
    pcl::toROSMsg(*differenceCloud, differenceCloudMsg);
//    differenceCloudMsg.header = std_msgs::Header();
//    differenceCloudMsg.header.stamp = currentCloudTimeStamp;
    differencesPub.publish(differenceCloudMsg);

    std::vector<pcl::PointXYZ> positions = getObjectPositions(differenceCloud);

    std::sort(positions.begin(), positions.end(), ObjectFinder::positionCompare);

    object_finder::Positions object_pos;
    object_pos.header = std_msgs::Header();
    object_pos.header.stamp = currentCloudTimeStamp;

    for(size_t i = 0; i < positions.size(); i++){
        geometry_msgs::Point point;
        point.x = positions[i].x;
        point.y = positions[i].y;
        point.z = positions[i].z;

        object_pos.object_positions.push_back(point);
        object_pos.object_radiuses.push_back(0.02);

        sendMarker(positions[i], i);
    }

    objectsPub.publish(object_pos);

    return;
}

pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::downSample(const pcl::PointCloud<POINTTYPE>::Ptr &pc){

    pcl::ApproximateVoxelGrid<POINTTYPE> grid;
    grid.setLeafSize(0.002, 0.002, 0.002);
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

//    for(size_t i = 0; i < projectedCloud->points.size(); i++){
//        if(!isnan(projectedCloud->points[i].y)){
//            ROS_INFO("setting y from %f to 0", pc->points[i].y);
//            pc->points[i].y = 0;
//        }
//    }
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

//    float resolution = 32.0f;
//    pcl::octree::OctreePointCloudChangeDetector<POINTTYPE> octree(resolution);

//    octree.setInputCloud(upc);
//    octree.addPointsFromInputCloud();

//    octree.switchBuffers();

//    octree.setInputCloud(lpc);
//    octree.addPointsFromInputCloud();

//    std::vector<int> newPointIdxVector;

//    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

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

std::vector<pcl::PointXYZ> ObjectFinder::getObjectPositions(const pcl::PointCloud<POINTTYPE>::Ptr &pc){

    pcl::search::KdTree<POINTTYPE>::Ptr kdTree (new pcl::search::KdTree<POINTTYPE>);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<POINTTYPE> ec;
    ec.setClusterTolerance (0.03); // 3cm
    ec.setMinClusterSize (300);
    ec.setMaxClusterSize (5000);
    ec.setSearchMethod (kdTree);
    ec.setInputCloud (pc);
    ec.extract(cluster_indices);

    std::vector<pcl::PointXYZ> objectPositions(0);

    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), end = cluster_indices.end(); it != end; it++){
        pcl::PointXYZ midPoint;
        midPoint.x = 0;
        midPoint.y = 0;
        for(std::vector<int>::const_iterator pit = it->indices.begin(), pend = it->indices.end(); pit != pend; pit++){
            midPoint.x += pc->points[*pit].x;
            midPoint.y += pc->points[*pit].y;
            midPoint.z += pc->points[*pit].z;
        }
        midPoint.x /= it->indices.size();
        midPoint.y /= it->indices.size();
        midPoint.z /= it->indices.size();
        ROS_INFO("object position: (%f, %f)", midPoint.x, midPoint.y);
        objectPositions.push_back(midPoint);
    }

    return objectPositions;
}

bool ObjectFinder::positionCompare(pcl::PointXYZ lhs, pcl::PointXYZ rhs){
    double lhsDist = std::sqrt(std::pow(lhs.x, 2) + std::pow(lhs.z, 2));
    double rhsDist = std::sqrt(std::pow(rhs.x, 2) + std::pow(rhs.z, 2));
    return lhsDist < rhsDist;
}

void ObjectFinder::sendMarker(pcl::PointXYZ point, int id){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = currentCloudTimeStamp;
    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    markerPub.publish(marker);
}

} //namespace primesense_pkgs
