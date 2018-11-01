/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 * @brief script to test cloud filtering and plane fitting
 */
#include <string>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
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
    ros::Publisher mPlaneMarkerPub;
    tf::TransformListener mTfListener;
    std::string mTargetFrame;
    bool mExtractPlanes;

public:
    CloudProcessingTestNode(const ros::NodeHandle &pNodeHandle, const std::string &pCloudTopic,
            const std::string &pProcessedCloudTopic, const std::string &pTargetFrame, bool pExtractPlanes)
    : mNodeHandle(pNodeHandle), mExtractPlanes(pExtractPlanes), mPlaneFittingConfigServer(mNodeHandle),
      mTargetFrame(pTargetFrame)
    {
        ROS_INFO("setting up dynamic reconfiguration server for fitting planes");
        auto pfCallback = boost::bind(&CloudProcessingTestNode::planeFittingConfigCallback, this, _1, _2);
        mPlaneFittingConfigServer.setCallback(pfCallback);

        ROS_INFO("subscribing to point cloud topic and advertising processed result");
        mFilteredCloudPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>(pProcessedCloudTopic, 1);
        mCloudSub = mNodeHandle.subscribe(pCloudTopic, 1, &CloudProcessingTestNode::cloudCallback, this);

        mPlaneMarkerPub = mNodeHandle.advertise<visualization_msgs::Marker>("detected_planes", 1);
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

        auto transformedCloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
        if (!pcl_ros::transformPointCloud(mTargetFrame, *pCloudMsgPtr, *transformedCloudPtr, mTfListener))
        {
            ROS_WARN("failed to transform cloud to frame '%s' from frame '%s'",
                     mTargetFrame.c_str(), pCloudMsgPtr->header.frame_id.c_str());
            return;
        }

        if (!mExtractPlanes)
        {
            // only do cloud filtering
            auto filteredCloudPtr = mPlaneSegmenter.filterCloud(transformedCloudPtr);
            mFilteredCloudPub.publish(*filteredCloudPtr);
            return;
        }

        // also fit plane
        auto filteredCloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
        mcr_perception_msgs::PlaneList::Ptr planeListPtr;
        try
        {
            planeListPtr = mPlaneSegmenter.findPlanes(transformedCloudPtr, filteredCloudPtr);
        }
        catch (std::runtime_error &ex)
        {
            ROS_ERROR("failed to find planes: %s", ex.what());
            return;
        }

        mFilteredCloudPub.publish(*filteredCloudPtr);
        if (planeListPtr->planes.empty())
        {
            ROS_ERROR("found no plane in point cloud");
            return;
        }
        auto firstPlane = planeListPtr->planes[0];
        ROS_INFO("plane center: (%.3f, %.3f, %.3f), limits: x~(%.3f, %.3f), y~(%.3f, %.3f), normal: (%.3f, %.3f, %.3f)",
                 firstPlane.plane_point.x, firstPlane.plane_point.y, firstPlane.plane_point.z,
                 firstPlane.limits.min_x, firstPlane.limits.max_x, firstPlane.limits.min_y, firstPlane.limits.max_y,
                 firstPlane.coefficients[0], firstPlane.coefficients[1], firstPlane.coefficients[2]);

        if (mPlaneMarkerPub.getNumSubscribers() == 0)
            return;

        auto planeMarkerPtr = planeMsgToMarkers(planeListPtr->planes[0], "planar_polygon");
        mPlaneMarkerPub.publish(*planeMarkerPtr);
    }
};

}   // namespace mas_perception_libs

int main(int pArgc, char** pArgv)
{
    ros::init(pArgc, pArgv, "cloud_processing_cpp_test");
    ros::NodeHandle nh("~");

    // load launch parameters
    bool extractPlanes;
    std::string cloudTopic, processedCloudTopic, targetFrame;
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
    if (!nh.getParam("target_frame", targetFrame) || targetFrame.empty())
    {
        ROS_ERROR("No 'target_frame' specified as parameter");
        return EXIT_FAILURE;
    }
    nh.param("extract_planes", extractPlanes, false);

    // run cloud filtering and plane segmentation
    mas_perception_libs::CloudProcessingTestNode cloudProcessingTestNode(nh, cloudTopic, processedCloudTopic,
                                                                         targetFrame, extractPlanes);

    while (ros::ok())
        ros::spin();

    return 0;
}
