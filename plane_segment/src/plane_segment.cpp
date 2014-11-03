#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

using std::vector;

class PlaneSegmenter{

private:

    ros::Subscriber sub;
    ros::Publisher pub;

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;
    int minimalPointsPerPlane;

    vector<uint32_t> colors;


    bool extractPlane(const pcl::PointCloud<POINTTYPE>::Ptr pc, pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers){

        coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);

        //downsample data
        ROS_INFO("Points before downsampling: %lu", pc->points.size());
        pcl::PointCloud<POINTTYPE>::Ptr dsCloud = downSample(pc);
        ROS_INFO("Points after downsampling: %lu", dsCloud->points.size());

        // Create the segmentation object
        pcl::SACSegmentation<POINTTYPE> seg;
        seg.setOptimizeCoefficients (true);

        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (dsCloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () < minimalPointsPerPlane)
        {
            //no plane detected
            return false;
        }
        else{

            //find all points of the original cloud that fit to the model

            Eigen::VectorXf modelVec(4);
            modelVec[0] = coefficients->values[0];
            modelVec[1] = coefficients->values[1];
            modelVec[2] = coefficients->values[2];
            modelVec[3] = coefficients->values[3];

            pcl::SampleConsensusModelPlane<POINTTYPE> scmp(pc);
            scmp.selectWithinDistance(modelVec, 0.005, inliers->indices);
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
        pub = nh.advertise<sensor_msgs::PointCloud2>("plane_segment/plane", 1);
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

        pcl::PointCloud<POINTTYPE> outputCloud(*inputCloud);
        //pcl::PointCloud<POINTTYPE> originalCloud(*inputCloud);

        inputCloud = extractBox(inputCloud);

        size_t colorIndex = 0;

        pcl::ExtractIndices<POINTTYPE> extractor;
        extractor.setKeepOrganized(true);

        while(extractPlane(inputCloud, coefficients, inliers)){

            ROS_INFO("Model coefficients: %f, %f, %f, %f",
                     coefficients->values[0],
                     coefficients->values[1],
                     coefficients->values[2],
                     coefficients->values[3]
                     );

            extractor.setIndices(inliers);
            extractor.setNegative(true);
            extractor.filterDirectly(inputCloud);

            //uint32_t rgb = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
            uint32_t rgb = colors[(colorIndex++) % colors.size()];
            //uint32_t rgb = 0;
            for(size_t i = 0; i < inliers->indices.size(); i++){

                //color detected plane red
                outputCloud.points[inliers->indices[i]].rgb = *reinterpret_cast<float*>(&rgb);

            }
        }
        sensor_msgs::PointCloud2 planeCloudMsg;
        pcl::toROSMsg(outputCloud, planeCloudMsg);
        pub.publish(planeCloudMsg);
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
