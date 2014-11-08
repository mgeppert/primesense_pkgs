#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <plane_segment/CloudPlanes.h>
#include <plane_segment/PlaneModel.h>

#include <dynamic_reconfigure/server.h>
#include <plane_segment/CropBoxParamsConfig.h>

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
//    ros::Publisher debugPub;
    ros::Publisher planePub;

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;

    pcl::CropBox<POINTTYPE> cropBox;

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

        sub = nh.subscribe("/cloud_preparation/prepared_cloud", 1, &PlaneSegmenter::pointCloudCallback, this);
//        debugPub = nh.advertise<sensor_msgs::PointCloud2>("plane_segment/debugOut", 1);
        planePub = nh.advertise<plane_segment::CloudPlanes>("plane_segment/planes", 1);
        inputCloud = pcl::PointCloud<POINTTYPE>::Ptr (new pcl::PointCloud<POINTTYPE>());
        timeStamp = ros::Time();

        cropBox = pcl::CropBox<POINTTYPE>();
        cropBox.setKeepOrganized(false);

        cropBox.setMin(Eigen::Vector4f(-10.0, -10.0, 0.0, 1.0));
        cropBox.setMax(Eigen::Vector4f(10.0, 10.0, 1.5, 1.0));

        minimalPointsPerPlane = 500;

        return;
    }

    void updateData(Eigen::Vector4f& cropBoxMin, Eigen::Vector4f& cropBoxMax, int minimumPointsPerPlane){
        cropBox.setMin(cropBoxMin);
        cropBox.setMax(cropBoxMax);
        minimalPointsPerPlane = minimumPointsPerPlane;

        ros::param::set("/plane_segment/min_x", cropBoxMin[0]);
        ros::param::set("/plane_segment/min_y", cropBoxMin[1]);
        ros::param::set("/plane_segment/min_z", cropBoxMin[2]);
        return;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

//        pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
//        pcl::fromROSMsg(*msg, tempCloud);

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
        planePub.publish(planeCloudMsg);

        return;
    }

};

Eigen::Vector4f cropBoxMin;
Eigen::Vector4f cropBoxMax;
int minimumPointsPerPlane;
bool dataChanged;

void paramCallback(plane_segment::CropBoxParamsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Cropbox: (%f %f %f), (%f, %f, %f)",
             config.min_x,
             config.min_y,
             config.min_z,
             config.max_x,
             config.max_y,
             config.max_z);
    ROS_INFO("Minimum points per plane: %d", config.minPoints);

    cropBoxMin = Eigen::Vector4f(config.min_x, config.min_y, config.min_z, 1.0);
    cropBoxMax = Eigen::Vector4f(config.max_x, config.max_y, config.max_z, 1.0);
    minimumPointsPerPlane = config.minPoints;

    dataChanged = true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "plane_segment");

    dynamic_reconfigure::Server<plane_segment::CropBoxParamsConfig> server;
    dynamic_reconfigure::Server<plane_segment::CropBoxParamsConfig>::CallbackType f;

    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);

    PlaneSegmenter segmenter;

    dataChanged = false;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();

        if(dataChanged){
            segmenter.updateData(cropBoxMin, cropBoxMax, minimumPointsPerPlane);
            dataChanged = false;
        }

        segmenter.segment();

        loop_rate.sleep();

    }
    return 0;
}
