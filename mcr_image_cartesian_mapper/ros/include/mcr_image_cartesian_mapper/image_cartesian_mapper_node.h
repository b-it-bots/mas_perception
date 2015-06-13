#ifndef IMAGECARTESIANMAPPERNODE_H_
#define IMAGECARTESIANMAPPERNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <string>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <Eigen/LU>

#define PI 3.14159265

class ImageCartesianMapperNode
{

    public:
        ImageCartesianMapperNode(ros::NodeHandle &nh);
        virtual ~ImageCartesianMapperNode();
        void eventCallback(const std_msgs::String &event_command);
        void poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose);
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info);
        
        void states();
        void initState();
        void idleState();
        void runState();
        bool cameraOpticalToCameraMetrical();
        bool cameraMetricalToCartesian();

    private:
        enum States {
            INIT,
            IDLE,
            RUNNING
        };

    private:
        ros::NodeHandle node_handler_;
        ros::Subscriber event_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber camera_info_sub_;
        sensor_msgs::CameraInfoConstPtr camera_info_;
        ros::Publisher event_pub_;
        ros::Publisher cartesian_pub_;
        bool pose_sub_status_;
        bool start_cartesian_mapper_;
        bool camera_info_sub_status_;
        geometry_msgs::Pose2D pose_2d_;
        geometry_msgs::PoseStamped camera_optical_pose_;
        geometry_msgs::PoseStamped cartesian_pose_;
        States run_state_;
        tf::TransformListener listener_;
        std::string target_frame_;
        std::string source_frame_;
        Eigen::Vector3f camera_coordinates_;
        Eigen::Matrix3f camera_intrinsic_matrix_;
        Eigen::Vector3f camera_metrical_coordinates_;
        std_msgs::String event_out_msg_;

};

#endif /* IMAGECARTESIANMAPPERNODE_H_ */
