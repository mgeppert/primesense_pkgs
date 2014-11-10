#include "ObjectFinder.h"

#include <object_finder/Objects.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>

namespace primesense_pkgs{

ObjectFinder::ObjectFinder(){

    ros::NodeHandle nh;

    sub = nh.subscribe("/cloud_preparation/prepared_cloud", 1, &ObjectFinder::cloudCallback, this);
    objectsPub = nh.advertise<object_finder::Objects>("object_finder/objects", 1);
    upperProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("object_finder/upper_projection", 1);
    lowerProjectionPub = nh.advertise<sensor_msgs::PointCloud2>("object_finder/lower_projection", 1);
    differencesPub = nh.advertise<sensor_msgs::PointCloud2>("object_finder/differences", 1);
    markerPub = nh.advertise<visualization_msgs::Marker>("object_finder/marker", 1);

    lowerBox = pcl::CropBox<POINTTYPE>();
    lowerBox.setMin(Eigen::Vector4f(-10.0, 0.01, 0.0, 1.0));
    lowerBox.setMax(Eigen::Vector4f(10.0, 0.1, 10.0, 1.0));

    upperBox = pcl::CropBox<POINTTYPE>();
    upperBox.setMin(Eigen::Vector4f(-10.0, 0.1, 0.0, 1.0));
    upperBox.setMax(Eigen::Vector4f(10.0, 0.2, 10.0, 1.0));

//    float resolution = 32.0f;
//    octree = pcl::octree::OctreePointCloudChangeDetector<POINTTYPE>::Ptr(new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>(resolution));

    inputCloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);
    currentCloudTimeStamp = ros::Time::now();

    return;
}

void ObjectFinder::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
//    ROS_INFO("received cloud");

    pcl::fromROSMsg(*msg, *inputCloud);
    return;
}

void ObjectFinder::findObjects(){

    pcl::PointCloud<POINTTYPE>::Ptr upperCloud = cropUpperBox(inputCloud);
    pcl::PointCloud<POINTTYPE>::Ptr lowerCloud = cropLowerBox(inputCloud);

    ROS_INFO("#points: upper: %lu, lower: %lu", upperCloud->points.size(), lowerCloud->points.size());

    pcl::PointCloud<POINTTYPE>::Ptr upperProjection = projectToZeroPlane(upperCloud);
    pcl::PointCloud<POINTTYPE>::Ptr lowerProjection = projectToZeroPlane(lowerCloud);
    ROS_INFO("lower projection points, before: %lu, after: %lu", lowerCloud->points.size(), lowerProjection->points.size());

    ROS_INFO("#points: upperProjection: %lu, lowerProjection: %lu", upperProjection->points.size(), lowerProjection->points.size());

    sensor_msgs::PointCloud2 upperProjectionMsg;
    pcl::toROSMsg(*upperProjection, upperProjectionMsg);
    upperProjectionPub.publish(upperProjectionMsg);

    sensor_msgs::PointCloud2 lowerProjectionMsg;
    pcl::toROSMsg(*lowerProjection, lowerProjectionMsg);
    lowerProjectionPub.publish(lowerProjectionMsg);

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

    if(positions.size() > 0){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_depth_optical_frame";
        marker.header.stamp = currentCloudTimeStamp;
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = positions[0].x;
        marker.pose.position.y = positions[0].y;
        marker.pose.position.z = positions[0].z;
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

    object_finder::Objects objects;
    objects.header = std_msgs::Header();
    objects.header.stamp = currentCloudTimeStamp;

    for(std::vector<pcl::PointXYZ>::const_iterator it = positions.begin(), end = positions.end(); it != end; it++){
        geometry_msgs::Point point;
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;

        objects.object_positions.push_back(point);
        objects.object_radiuses.push_back(0.02);
    }

    objectsPub.publish(objects);

    return;
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

//pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::removeGroundPlane(const pcl::PointCloud<POINTTYPE>::Ptr &pc){

//}

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

//pcl::PointCloud<POINTTYPE>::Ptr ObjectFinder::getDifferenceCloud(const pcl::PointCloud<POINTTYPE>::Ptr &lpc, const std::vector<int> &differenceIndices){

//    pcl::PointCloud<POINTTYPE>::Ptr differenceCloud(new pcl::PointCloud<POINTTYPE>);
////    differenceCloud->points.resize(differenceIndices.size());

////    for(size_t i = 0; i < differenceIndices.size(); i++){
////        differenceCloud->points[i] = lpc->points[differenceIndices[i]];
////    }
//    pcl::ExtractIndices<POINTTYPE> extractor;

//    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
//    indices->indices = differenceIndices;
//    extractor.setIndices(indices);
//    extractor.setInputCloud(lpc);
//    extractor.filter(*differenceCloud);
//    return differenceCloud;
//}

std::vector<pcl::PointXYZ> ObjectFinder::getObjectPositions(const pcl::PointCloud<POINTTYPE>::Ptr &pc){

    pcl::search::KdTree<POINTTYPE>::Ptr kdTree (new pcl::search::KdTree<POINTTYPE>);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<POINTTYPE> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (200);
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

} //namespace primesense_pkgs
