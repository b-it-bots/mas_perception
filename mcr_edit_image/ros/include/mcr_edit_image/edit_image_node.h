#ifndef EDITIMAGENODE_H_
#define EDITIMAGENODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <mcr_edit_image/EditImageConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class EditImageNode
{

    public:
        EditImageNode(ros::NodeHandle &nh);
        virtual ~EditImageNode();
        void dynamicReconfigCallback(mcr_edit_image::EditImageConfig &config, uint32_t level);
        void eventCallback(const std_msgs::String &event_command);
        void imageCallback(const sensor_msgs::ImageConstPtr &image_message);
        void states();
        void initState();
        void idleState();
        void runState();
        void editImage();

    private:
        enum States {
            INIT,
            IDLE,
            RUNNING
        };

    private:
        ros::NodeHandle node_handler_;
        dynamic_reconfigure::Server<mcr_edit_image::EditImageConfig> dynamic_reconfig_server_;
        ros::Subscriber event_sub_;
        image_transport::Subscriber image_sub_;
        image_transport::ImageTransport image_transporter_;
        image_transport::Publisher image_pub_;
        bool image_sub_status_;
        sensor_msgs::ImageConstPtr image_message_;
        bool is_rotation_;
        bool is_crop_;
        bool start_edit_image_;
        double crop_factor_top_;
        double crop_factor_bottom_;
        double crop_factor_left_;
        double crop_factor_right_;
        States run_state_;

};

#endif /* EDITIMAGENODE_H_ */