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
#include <mas_perception_libs/CloudFilterConfig.h>
#include <mas_perception_libs/point_cloud_utils.h>
#include <mas_perception_libs/point_cloud_utils_ros.h>
#include <mas_perception_libs/sac_plane_segmenter.h>

namespace mas_perception_libs
{

class PlaneSegmenterTestNode
{
private:
    ros::NodeHandle mNodeHandle;
    dynamic_reconfigure::Server<CloudFilterConfig> mCloudFilterConfigServer;
    CloudFilterROS mCloudFilter;
    ros::Subscriber mCloudSub;
    ros::Publisher mFilteredCloudPub;

public:
    PlaneSegmenterTestNode(const ros::NodeHandle &pNodeHandle, const std::string &pCloudTopic,
            const std::string &pFilteredCloudTopic)
    : mNodeHandle(pNodeHandle)
    {
        ROS_INFO("setting up dynamic reconfiguration server for filtering point clouds");
        dynamic_reconfigure::Server<CloudFilterConfig>::CallbackType f =
                boost::bind(&PlaneSegmenterTestNode::cloudFilterConfigCallback, this, _1, _2);
        mCloudFilterConfigServer.setCallback(f);

        ROS_INFO("subscribing to point cloud topic and advertising filter result");
        mFilteredCloudPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>(pFilteredCloudTopic, 1);
        mCloudSub = mNodeHandle.subscribe(pCloudTopic, 1, &PlaneSegmenterTestNode::cloudCallback, this);
    }

private:
    void
    cloudFilterConfigCallback(const CloudFilterConfig &pConfig, uint32_t pLevel)
    {
        mCloudFilter.setParams(pConfig);
    }

    void
    cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pCloudMsgPtr)
    {
        // do not process cloud when there's no subscriber
        if (mFilteredCloudPub.getNumSubscribers() == 0)
            return;

        auto filteredCloudMsg = mCloudFilter.filterCloud(pCloudMsgPtr);;
        mFilteredCloudPub.publish(*filteredCloudMsg);
    }
};

}   // namespace mas_perception_libs

int main(int pArgc, char** pArgv)
{
    ros::init(pArgc, pArgv, "plane_segmenter_test_node");
    ros::NodeHandle nh("~");

    // load launch parameters
    std::string cloudTopic, filteredCloudTopic;
    if (!nh.getParam("cloud_topic", cloudTopic) || cloudTopic.empty())
    {
        ROS_ERROR("No 'cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("filtered_cloud_topic", filteredCloudTopic) || filteredCloudTopic.empty())
    {
        ROS_ERROR("No 'filteredcloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }

    // run cloud filtering and plane segmentation
    mas_perception_libs::PlaneSegmenterTestNode segmenterTestNode(nh, cloudTopic, filteredCloudTopic);

    while (ros::ok())
        ros::spin();
    return 0;
}
