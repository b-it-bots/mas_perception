#ifndef BLOBTRACKINGERRORMONITORNODE_H_
#define BLOBTRACKINGERRORMONITORNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <mcr_manipulation_msgs/ComponentWiseCartesianDifference.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <mcr_perception_monitors/BlobTrackingErrorMonitorConfig.h>
#include <math.h>


class BlobTrackingErrorMonitorNode
{

    public:
        BlobTrackingErrorMonitorNode(ros::NodeHandle &nh);
        virtual ~BlobTrackingErrorMonitorNode();
        void dynamicReconfigCallback(mcr_perception_monitors::BlobTrackingErrorMonitorConfig &config, uint32_t level);
        void eventCallback(const std_msgs::String &event_command);
        void errorCallback(const mcr_manipulation_msgs::ComponentWiseCartesianDifference::ConstPtr &pose_error);
        void states();
        void initState();
        void idleState();
        void runState();
        bool isBlobTrackingErrorWithinThreshold();

    private:
        enum States {
            INIT,
            IDLE,
            RUNNING
        };

    private:
        ros::NodeHandle node_handler_;
        dynamic_reconfigure::Server<mcr_perception_monitors::BlobTrackingErrorMonitorConfig> dynamic_reconfig_server_;
        ros::Subscriber event_sub_;
        ros::Subscriber error_sub_;
        ros::Publisher event_pub_;
        bool tracking_error_sub_status_;
        bool start_tracking_error_monitor_;
        double threshold_linear_x_;
        double threshold_linear_y_;
        double threshold_linear_z_;
        double threshold_angular_x_;
        double threshold_angular_y_;
        double threshold_angular_z_;
        mcr_manipulation_msgs::ComponentWiseCartesianDifference error_;
        std_msgs::String status_msg_;
        States run_state_;

};

#endif /* BLOBTRACKINGERRORMONITORNODE_H_ */
