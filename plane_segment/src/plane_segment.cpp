#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <plane_segment/CloudPlanes.h>
#include <plane_segment/PlaneModel.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/search/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

using std::vector;

class PlaneSegmenter{

private:

    ros::Subscriber sub;
    ros::Publisher debugPub;
    ros::Publisher planePub;

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;
    int minimalPointsPerPlane;

    vector<uint32_t> colors;


    bool extractPlane(const pcl::PointCloud<POINTTYPE>::Ptr pc, pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers){

        coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);

        //downsample data
        //ROS_INFO("Points before downsampling: %lu", pc->points.size());
        //pcl::PointCloud<POINTTYPE>::Ptr dsCloud = downSample(pc);
        //ROS_INFO("Points after downsampling: %lu", dsCloud->points.size());

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
        grid.setLeafSize(0.005, 0.005, 0.005);

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

        minimalPointsPerPlane = 200;

        colors = vector<uint32_t>(6, 0);
        colors[0] = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
        colors[1] = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        colors[2] = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        colors[3] = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        colors[4] = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        colors[5] = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

        pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
        pcl::fromROSMsg(*msg, tempCloud);

        inputCloud = tempCloud.makeShared();
//        pcl::fromROSMsg(*msg, *inputCloud);

        return;
    }

    void segment(){

        pcl::ModelCoefficients::Ptr coefficients;
        pcl::PointIndices::Ptr inliers;

        pcl::PointCloud<POINTTYPE>::Ptr outputCloud = inputCloud->makeShared();
        //pcl::PointCloud<POINTTYPE> originalCloud(*inputCloud);

        inputCloud = extractBox(inputCloud);

        size_t colorIndex = 0;

        //downsample data
        ROS_INFO("Points before downsampling: %lu", inputCloud->points.size());
        pcl::PointCloud<POINTTYPE>::Ptr dsCloud = downSample(inputCloud);
        ROS_INFO("Points after downsampling: %lu", dsCloud->points.size());

        pcl::ExtractIndices<POINTTYPE> extractor;
        extractor.setKeepOrganized(true);

        //std::vector<std::vector<float> > planeModels(0);

        //create Message to store data
        plane_segment::CloudPlanes planeCloudMsg;

        while(extractPlane(dsCloud, coefficients, inliers)){

            ROS_INFO("Model coefficients: %f, %f, %f, %f",
                     coefficients->values[0],
                     coefficients->values[1],
                     coefficients->values[2],
                     coefficients->values[3]
                     );

            //planeModels.push_back(coefficients->values);

            plane_segment::PlaneModel planeModelMsg;
            planeModelMsg.xParam = coefficients->values[0];
            planeModelMsg.yParam = coefficients->values[1];
            planeModelMsg.zParam = coefficients->values[2];
            planeModelMsg.dParam = coefficients->values[3];

            planeCloudMsg.planes.push_back(planeModelMsg);

            extractor.setIndices(inliers);
            extractor.setNegative(true);
            extractor.filterDirectly(dsCloud);

            //find all points of the original cloud that fit to the model

            Eigen::VectorXf modelVec(4);
            modelVec[0] = coefficients->values[0];
            modelVec[1] = coefficients->values[1];
            modelVec[2] = coefficients->values[2];
            modelVec[3] = coefficients->values[3];

            pcl::SampleConsensusModelPlane<POINTTYPE> scmp(outputCloud);
            scmp.selectWithinDistance(modelVec, 0.01, inliers->indices);

//            extractor.setIndices(inliers);
//            extractor.setNegative(true);
//            extractor.filterDirectly(outputCloud);

            //uint32_t rgb = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
            uint32_t rgb = colors[(colorIndex++) % colors.size()];
            //uint32_t rgb = 0;
            for(size_t i = 0; i < inliers->indices.size(); i++){

                //color detected plane
                outputCloud->points[inliers->indices[i]].rgb = *reinterpret_cast<float*>(&rgb);

            }
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

        sensor_msgs::PointCloud2 debugMsg;
        pcl::toROSMsg(*outputCloud, debugMsg);
        debugPub.publish(debugMsg);


//        std::vector<plane_segment::PlaneModel> planeMsgs(planeModels.size());

//        for(size_t i = 0; i < planeModels.size(); i++){
//            planeMsgs[i].planeCoefficients = planeModels[i];
//        }

//        planeCloudMsg.planes = planeMsgs;
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
