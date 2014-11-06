#include <ros/ros.h>

//messages
#include <sensor_msgs/PointCloud2.h>
#include <plane_segment/CloudPlanes.h>
#include <plane_segment/PlaneModel.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

namespace ObjectDetector{

using message_filters::Subscriber;
using message_filters::TimeSynchronizer;
using sensor_msgs::PointCloud2;
using plane_segment::CloudPlanes;

class ObjectDetector{

private:
    Subscriber<PointCloud2>* cloudSub;
    Subscriber<CloudPlanes>* planeSub;
    TimeSynchronizer<PointCloud2, CloudPlanes>* synchronizer;
    ros::Publisher pub;

    pcl::PointCloud<POINTTYPE>::Ptr cloud;

    std::vector<Eigen::VectorXf> planeModels;

    void removePlanes(){

        for(size_t i = 0; i < planeModels.size(); i++){

            std::vector<int> planeIndices;

            pcl::SampleConsensusModelPlane<POINTTYPE> scmp(cloud);
            scmp.selectWithinDistance(planeModels[i], 0.02, planeIndices);

            pcl::PointIndices::Ptr plane(new pcl::PointIndices);
            plane->indices = planeIndices;

            pcl::ExtractIndices<POINTTYPE> extractor;

            extractor.setIndices(plane);
            extractor.setNegative(true);
            extractor.filterDirectly(cloud);
        }

    }

    std::vector<pcl::PointXYZ> findComponents(){

        //TODO: compute centers

        //use kdtree to find connected components

        pcl::PointCloud<POINTTYPE>::Ptr sparseCloud(new pcl::PointCloud<POINTTYPE>);
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud, *sparseCloud, index);
        int minClusterSize = 500;
        pcl::search::KdTree<POINTTYPE>::Ptr tree (new pcl::search::KdTree<POINTTYPE>);
        tree->setInputCloud (sparseCloud);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<POINTTYPE> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (minClusterSize);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud (sparseCloud);

        ec.extract(cluster_indices);

        ROS_INFO("Number of clusters: %lu", cluster_indices.size());

        for(size_t i = 0; i < cluster_indices.size(); i++){

//            pcl::PointCloud<POINTTYPE> cluster;

////            pcl::PointIndices::Ptr clusterIndices(new pcl::PointIndices);
////            clusterIndices->indices = cluster_indices[i];

//            pcl::ExtractIndices<POINTTYPE> extractor;

//            pcl::PointIndicesConstPtr indicesPtr(cluster_indices[i]);

//            extractor.setIndices(indicesPtr);
//            extractor.setNegative(false);
//            extractor.setInputCloud(sparseCloud);
//            extractor.filter(cluster);

            std::vector<double> clusterMean(3, 0);
            for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin (); pit != cluster_indices[i].indices.end (); pit++){
                clusterMean[0] += sparseCloud->points[*pit].x;
                clusterMean[1] += sparseCloud->points[*pit].y;
                clusterMean[2] += sparseCloud->points[*pit].z;
            }

            ROS_INFO("Cluster mean: %f, %f, %f", clusterMean[0] / cluster_indices.size(), clusterMean[1] / cluster_indices.size(), clusterMean[2] / cluster_indices.size());


        }

        return std::vector<pcl::PointXYZ>();
    }

public:
    ObjectDetector(){

        ros::NodeHandle nh;

        cloudSub = new Subscriber<PointCloud2>(nh, "/camera/depth_registered/points", 1);
        cloudSub->registerCallback(&ObjectDetector::cloudCallback, this);
        planeSub = new Subscriber<CloudPlanes>(nh, "/plane_segment/planes", 1);
        planeSub->registerCallback(&ObjectDetector::planeCallback, this);

        synchronizer = new TimeSynchronizer<PointCloud2, CloudPlanes>(*cloudSub, *planeSub, 20);
        synchronizer->registerCallback(&ObjectDetector::cloudPlaneCallback, this);

        pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_visualizer/cloud", 1);

        cloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);
    }

    ~ObjectDetector(){
        delete cloudSub;
        delete planeSub;
        delete synchronizer;
    }

    void cloudCallback(const PointCloud2::ConstPtr& cloudMsg){
//        ROS_INFO("received cloud");
    }


    void planeCallback(const plane_segment::CloudPlanes::ConstPtr& planeMsg){
//        ROS_INFO("received planes");
    }

    void cloudPlaneCallback(const PointCloud2::ConstPtr& cloudMsg, const CloudPlanes::ConstPtr& planeMsg){

        ROS_INFO("received new cloud");

        if(!ros::ok()){
            ROS_INFO("ABORTING CALLBACK");
            return;
        }

        pcl::fromROSMsg(*cloudMsg, *cloud);

        planeModels = std::vector<Eigen::VectorXf>(planeMsg->planes.size());
        for(size_t i = 0; i < planeMsg->planes.size(); i++){
            Eigen::VectorXf modelVec(4);
            modelVec[0] = planeMsg->planes[i].xParam;
            modelVec[1] = planeMsg->planes[i].yParam;
            modelVec[2] = planeMsg->planes[i].zParam;
            modelVec[3] = planeMsg->planes[i].dParam;

            planeModels[i] = modelVec;
        }
    }

    void detectObjects(){

        //TODO: remove planes + compute centers of remaining (connected) surfaces;

        removePlanes();

        std::vector<pcl::PointXYZ> objects = findComponents();
    }
};
} //namespace ObjectDetector

int main(int argc, char** argv){

    ros::init(argc, argv, "object_detector_node");

    ObjectDetector::ObjectDetector objectDetector;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();

        objectDetector.detectObjects();

        loop_rate.sleep();
    }

    return 0;
}
