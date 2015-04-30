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
#include <std_msgs/UInt32.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#define PI 3.14159265

using namespace cv;


class ImageCartesianMapperNode
{

    public:
        ImageCartesianMapperNode(ros::NodeHandle &nh);
        virtual ~ImageCartesianMapperNode();
        void eventCallback(const std_msgs::String &event_command);
        void imageCallback(const sensor_msgs::ImageConstPtr &image);
        void poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose);
        void states();
        void initState();
        void idleState();
        void runState();
        void imagetoCartesianMapper();

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
        image_transport::Subscriber image_sub_;
        sensor_msgs::ImageConstPtr image_message_;
        image_transport::ImageTransport image_transporter_;
        ros::Publisher event_pub_;
        ros::Publisher cartesian_pub_;
        bool image_sub_status_;
        bool pose_sub_status_;
        bool start_cartesian_mapper_;
        geometry_msgs::Pose2D pose_2d_;
        geometry_msgs::PoseStamped cartesian_pose_;
        int center_x_;
        int center_y_;
        std_msgs::String status_msg_;
        States run_state_;

};

#endif /* IMAGECARTESIANMAPPERNODE_H_ */
