/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 * @brief script to test cloud filtering and plane fitting
 */
#include <string>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mas_perception_libs/aliases.h>
#include <mas_perception_libs/PlaneFittingConfig.h>
#include <mas_perception_libs/point_cloud_utils.h>
#include <mas_perception_libs/point_cloud_utils_ros.h>
#include <mas_perception_libs/sac_plane_segmenter.h>

namespace mas_perception_libs
{

class CloudProcessingTestNode
{
private:
    ros::NodeHandle mNodeHandle;
    dynamic_reconfigure::Server<PlaneFittingConfig> mPlaneFittingConfigServer;
    PlaneSegmenterROS mPlaneSegmenter;
    ros::Subscriber mCloudSub;
    ros::Publisher mFilteredCloudPub;
    bool mExtractPlanes;

public:
    CloudProcessingTestNode(const ros::NodeHandle &pNodeHandle, const std::string &pCloudTopic,
            const std::string &pProcessedCloudTopic, bool pExtractPlanes)
    : mNodeHandle(pNodeHandle), mExtractPlanes(pExtractPlanes), mPlaneFittingConfigServer(mNodeHandle)
    {
        ROS_INFO("setting up dynamic reconfiguration server for fitting planes");
        auto pfCallback = boost::bind(&CloudProcessingTestNode::planeFittingConfigCallback, this, _1, _2);
        mPlaneFittingConfigServer.setCallback(pfCallback);

        ROS_INFO("subscribing to point cloud topic and advertising processed result");
        mFilteredCloudPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>(pProcessedCloudTopic, 1);
        mCloudSub = mNodeHandle.subscribe(pCloudTopic, 1, &CloudProcessingTestNode::cloudCallback, this);
    }

private:
    void
    planeFittingConfigCallback(const PlaneFittingConfig &pConfig, uint32_t pLevel)
    {
        mPlaneSegmenter.setParams(pConfig);
    }

    void
    cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pCloudMsgPtr)
    {
        // do not process cloud when there's no subscriber
        if (mFilteredCloudPub.getNumSubscribers() == 0)
            return;

        if (!mExtractPlanes)
        {
            // only do cloud filtering
            auto filteredCloudPtr = mPlaneSegmenter.filterCloud(pCloudMsgPtr);
            mFilteredCloudPub.publish(*filteredCloudPtr);
            return;
        }

        // also fit plane
        PointCloud::Ptr hullPointsPtr = boost::make_shared<PointCloud>();
        pcl::ModelCoefficients::Ptr planeCoeffsPtr = boost::make_shared<pcl::ModelCoefficients>();
        double planeHeight = 0.0;
        mPlaneSegmenter.findPlane(pCloudMsgPtr, hullPointsPtr, planeCoeffsPtr, planeHeight);
        ROS_INFO("plane height: %.3f, hull size: %ld, normal: (%.3f, %.3f, %.3f)", planeHeight, hullPointsPtr->size(),
                 planeCoeffsPtr->values[0], planeCoeffsPtr->values[1], planeCoeffsPtr->values[2]);
    }
};

}   // namespace mas_perception_libs

int main(int pArgc, char** pArgv)
{
    ros::init(pArgc, pArgv, "cloud_processing_cpp_test");
    ros::NodeHandle nh("~");

    // load launch parameters
    bool extractPlanes;
    std::string cloudTopic, processedCloudTopic;
    if (!nh.getParam("cloud_topic", cloudTopic) || cloudTopic.empty())
    {
        ROS_ERROR("No 'cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("processed_cloud_topic", processedCloudTopic) || processedCloudTopic.empty())
    {
        ROS_ERROR("No 'processed_cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    nh.param("extract_planes", extractPlanes, false);

    // run cloud filtering and plane segmentation
    mas_perception_libs::CloudProcessingTestNode cloudProcessingTestNode(nh, cloudTopic, processedCloudTopic,
                                                                         extractPlanes);

    while (ros::ok())
        ros::spin();
    return 0;
}
