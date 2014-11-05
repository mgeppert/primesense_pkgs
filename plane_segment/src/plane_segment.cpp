#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <plane_segment/CloudPlanes.h>
#include <plane_segment/PlaneModel.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/search/kdtree.h>

#include <vector>

#define POINTTYPE pcl::PointXYZ

using std::vector;

class PlaneSegmenter{

private:

    ros::Subscriber sub;
    ros::Publisher debugPub;
    ros::Publisher planePub;

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;

    ros::Time timeStamp;

    int minimalPointsPerPlane;

    vector<uint32_t> colors;

    //find the biggest plane in the pointcloud
    bool extractPlane(const pcl::PointCloud<POINTTYPE>::Ptr pc, pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers){

        coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<POINTTYPE> seg;
        seg.setOptimizeCoefficients (true);

        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.015);

        seg.setInputCloud (pc);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () < minimalPointsPerPlane)
        {
            //no plane detected
            return false;
        }
        else{
            return true;
        }
    }

    //cut out a box out of the pointcloud
    pcl::PointCloud<POINTTYPE>::Ptr extractBox(pcl::PointCloud<POINTTYPE>::Ptr pc){

        pcl::CropBox<POINTTYPE> cropBox;

        cropBox.setMin(Eigen::Vector4f(-10.0, -10.0, 0.0, 1.0));
        cropBox.setMax(Eigen::Vector4f(10.0, 10.0, 1.5, 1.0));
        cropBox.setKeepOrganized(true);

        cropBox.setInputCloud(pc);

        pcl::PointCloud<POINTTYPE>::Ptr croppedCloud(new pcl::PointCloud<POINTTYPE>());
        cropBox.filter(*croppedCloud);

        return croppedCloud;
    }

    //create a downsampled copy of the input cloud
    pcl::PointCloud<POINTTYPE>::Ptr downSample(pcl::PointCloud<POINTTYPE>::Ptr pc){

        pcl::ApproximateVoxelGrid<POINTTYPE> grid;
        grid.setLeafSize(0.01, 0.01, 0.01);

        grid.setInputCloud(pc);

        pcl::PointCloud<POINTTYPE>::Ptr filterOutput(new pcl::PointCloud<POINTTYPE>());
        grid.filter(*filterOutput);

        return filterOutput;
    }

public:
    PlaneSegmenter(){

        ros::NodeHandle nh;

        sub = nh.subscribe("/camera/depth_registered/points", 1, &PlaneSegmenter::pointCloudCallback, this);
        debugPub = nh.advertise<sensor_msgs::PointCloud2>("plane_segment/debugOut", 1);
        planePub = nh.advertise<plane_segment::CloudPlanes>("plane_segment/planes", 1);
        inputCloud = pcl::PointCloud<POINTTYPE>::Ptr (new pcl::PointCloud<POINTTYPE>());
        timeStamp = ros::Time();

        minimalPointsPerPlane = 500;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

        pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
        pcl::fromROSMsg(*msg, tempCloud);

        timeStamp = msg->header.stamp;
        pcl::fromROSMsg(*msg, *inputCloud);

        return;
    }

    void segment(){

        pcl::ModelCoefficients::Ptr coefficients;
        pcl::PointIndices::Ptr inliers;

        inputCloud = extractBox(inputCloud);

        //downsample data
//        ROS_INFO("Points before downsampling: %lu", inputCloud->points.size());
        pcl::PointCloud<POINTTYPE>::Ptr dsCloud = downSample(inputCloud);
//        ROS_INFO("Points after downsampling: %lu", dsCloud->points.size());

        pcl::ExtractIndices<POINTTYPE> extractor;
        extractor.setKeepOrganized(true);

        //create Message to store data
        plane_segment::CloudPlanes planeCloudMsg;
        planeCloudMsg.header.stamp = timeStamp;

        while(extractPlane(dsCloud, coefficients, inliers)){

            ROS_INFO("Model coefficients: %f, %f, %f, %f",
                     coefficients->values[0],
                     coefficients->values[1],
                     coefficients->values[2],
                     coefficients->values[3]
                     );

            plane_segment::PlaneModel planeModelMsg;
            planeModelMsg.xParam = coefficients->values[0];
            planeModelMsg.yParam = coefficients->values[1];
            planeModelMsg.zParam = coefficients->values[2];
            planeModelMsg.dParam = coefficients->values[3];

            planeCloudMsg.planes.push_back(planeModelMsg);

            extractor.setIndices(inliers);
            extractor.setNegative(true);
            extractor.filterDirectly(dsCloud);
        }

//        //use kdtree to find connected components

//        pcl::PointCloud<POINTTYPE>::Ptr sparseCloud(new pcl::PointCloud<POINTTYPE>);
//        pcl::removeNaNFromPointCloud(*outputCloud, *sparseCloud, std::vector<int>());
//        int minClusterSize = 50;
//        pcl::search::KdTree<POINTTYPE>::Ptr tree (new pcl::search::KdTree<POINTTYPE>);
//        tree->setInputCloud (sparseCloud);

//        std::vector<pcl::PointIndices> cluster_indices;

//        pcl::EuclideanClusterExtraction<POINTTYPE> ec;
//        ec.setClusterTolerance (0.02); // 2cm
//        ec.setMinClusterSize (minClusterSize);
//        ec.setMaxClusterSize (25000);
//        ec.setSearchMethod(tree);
//        ec.setInputCloud (sparseCloud);

//        ec.extract(cluster_indices);

//        ROS_INFO("Number of clusters: %lu", cluster_indices.size());

//        std::vector<double> clusterMean(3, 0.0);

//        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
//            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
//                clusterMean[0] += sparseCloud->points[*pit].x;
//                clusterMean[1] += sparseCloud->points[*pit].y;
//                clusterMean[2] += sparseCloud->points[*pit].z;
//            }

//            ROS_INFO("Cluster mean: %f, %f, %f", clusterMean[0] / cluster_indices.size(), clusterMean[1] / cluster_indices.size(), clusterMean[2] / cluster_indices.size());

////            ROS_INFO("Points before extraction: %lu", inputCloud->points.size());
////            pcl::ExtractIndices<POINTTYPE> extractor;
////            extractor.setNegative(true);
////            //pcl::PointIndices::Ptr inds;
////            //inds->indices = cluster_indices;
////            extractor.setIndices(it);
////            extractor.filterDirectly(inputCloud);
////            ROS_INFO("Points after extraction: %lu", inputCloud->points.size());
//        }

//        sensor_msgs::PointCloud2 debugMsg;
//        pcl::toROSMsg(*outputCloud, debugMsg);
//        debugPub.publish(debugMsg);


//        std::vector<plane_segment::PlaneModel> planeMsgs(planeModels.size());

//        for(size_t i = 0; i < planeModels.size(); i++){
//            planeMsgs[i].planeCoefficients = planeModels[i];
//        }

//        planeCloudMsg.planes = planeMsgs;
//        ROS_INFO("sending plane message with timestamp %d, %d", planeCloudMsg.header.stamp.sec, planeCloudMsg.header.stamp.nsec);
        planePub.publish(planeCloudMsg);
    }

};

int main(int argc, char **argv){

    ros::init(argc, argv, "plane_segment");

    PlaneSegmenter segmenter;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();

        segmenter.segment();

        loop_rate.sleep();

    }
    return 0;
}
