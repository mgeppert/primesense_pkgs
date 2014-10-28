#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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

        // Create the segmentation object
        pcl::SACSegmentation<POINTTYPE> seg;
        seg.setOptimizeCoefficients (true);

        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.005);

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

public:
    PlaneSegmenter(){

        ros::NodeHandle nh;

        sub = nh.subscribe("/camera/depth_registered/points", 1, &PlaneSegmenter::pointCloudCallback, this);
        pub = nh.advertise<sensor_msgs::PointCloud2>("plane_segment/plane", 1);
        inputCloud = pcl::PointCloud<POINTTYPE>::Ptr (new pcl::PointCloud<POINTTYPE>());

        minimalPointsPerPlane = 20000;

        colors = vector<uint32_t>(3, 0);
        colors[0] = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
        colors[1] = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        colors[2] = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
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
            uint32_t rgb = colors[(colorIndex++) % 3];
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
