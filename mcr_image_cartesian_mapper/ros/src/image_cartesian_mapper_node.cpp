#include <mcr_image_cartesian_mapper/image_cartesian_mapper_node.h>

ImageCartesianMapperNode::ImageCartesianMapperNode(ros::NodeHandle &nh) : node_handler_(nh)
{
    event_sub_ = node_handler_.subscribe("event_in", 1, &ImageCartesianMapperNode::eventCallback, this);
    pose_sub_ = node_handler_.subscribe("pose", 1, &ImageCartesianMapperNode::poseCallback, this);
    camera_info_sub_ = node_handler_.subscribe("camera_info", 1, &ImageCartesianMapperNode::cameraInfoCallback, this);
    cartesian_pub_ = node_handler_.advertise<geometry_msgs::PoseStamped>("cartesian_pose", 1);
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    node_handler_.param<std::string>("target_frame", target_frame_, "base_link");
    node_handler_.param<std::string>("source_frame", source_frame_, "tower_cam3d_rgb_optical_frame");
    run_state_ = INIT;
    start_cartesian_mapper_ = false;
    pose_sub_status_ = false;
    camera_info_sub_status_ = false;
}

ImageCartesianMapperNode::~ImageCartesianMapperNode()
{
    event_sub_.shutdown();
    pose_sub_.shutdown();
    camera_info_sub_.shutdown();
    cartesian_pub_.shutdown();
    event_pub_.shutdown();  
}


void ImageCartesianMapperNode::eventCallback(const std_msgs::String &event_command)
{
    if (event_command.data == "e_start") {
        start_cartesian_mapper_ = true;
        ROS_INFO("ENABLED");
    } else if (event_command.data == "e_stop") {
        start_cartesian_mapper_ = false;
        ROS_INFO("DISABLED");
    }
}

void ImageCartesianMapperNode::poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose)
{
    pose_2d_ = *pose;
    pose_sub_status_ = true;
}

void ImageCartesianMapperNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    camera_info_ = camera_info;
    camera_info_sub_status_ = true;
}

void ImageCartesianMapperNode::states()
{
    switch (run_state_) {
        case INIT:
            initState();
            break;
        case IDLE:
            idleState();
            break;
        case RUNNING:
            runState();
            break;
        default:
            initState();
    }
}

void ImageCartesianMapperNode::initState()
{
    if (start_cartesian_mapper_) {
        run_state_ = IDLE;
    } else {
        run_state_ = INIT;
    }

}

void ImageCartesianMapperNode::idleState()
{
    if (start_cartesian_mapper_) {
        if (pose_sub_status_ && camera_info_sub_status_) {
            run_state_ = RUNNING;
            pose_sub_status_ = false;
            camera_info_sub_status_ = false;
        } else {
            run_state_ = IDLE;
        }
    } else {
        run_state_ = INIT;
    }
}

void ImageCartesianMapperNode::runState()
{

    if(cameraOpticalToCameraMetrical()){
        if(cameraMetricalToCartesian()){
            cartesian_pub_.publish(cartesian_pose_);    
        } else {
            event_out_msg_.data = "e_error";
            event_pub_.publish(event_out_msg_);
        }
    }

    if (start_cartesian_mapper_) {
        run_state_ = IDLE;
    } else {
        run_state_ = INIT;
    }

}


bool ImageCartesianMapperNode::cameraOpticalToCameraMetrical()
{

    camera_coordinates_ << pose_2d_.x, pose_2d_.y, 1;

    camera_intrinsic_matrix_ << camera_info_->K[0], camera_info_->K[1], camera_info_->K[2], 
                                camera_info_->K[3], camera_info_->K[4], camera_info_->K[5], 
                                camera_info_->K[6], camera_info_->K[7], camera_info_->K[8];

    camera_metrical_coordinates_ = camera_intrinsic_matrix_.inverse()*camera_coordinates_;

    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;

    point.x = camera_metrical_coordinates_(0,0);
    point.y = camera_metrical_coordinates_(1,0);
    point.z = camera_metrical_coordinates_(2,0);
    quaternion = tf::createQuaternionMsgFromYaw(pose_2d_.theta*M_PI/180);
    pose.position = point;
    pose.orientation = quaternion;

    camera_optical_pose_.header.stamp = ros::Time::now();
    camera_optical_pose_.header.frame_id = source_frame_;
    camera_optical_pose_.pose = pose;

    return true;
}


bool ImageCartesianMapperNode::cameraMetricalToCartesian()
{
    try{
        listener_.waitForTransform(target_frame_, camera_optical_pose_.header.frame_id, camera_optical_pose_.header.stamp, ros::Duration(3.0));
        listener_.transformPose(target_frame_, camera_optical_pose_, cartesian_pose_);
        return true;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_cartesian_mapper");
    ros::NodeHandle nh("~");
    ROS_INFO("Initialised");
    ImageCartesianMapperNode icm(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);
    
    while (ros::ok()) {
        ros::spinOnce();
        icm.states();
        rate.sleep();
    }

    return 0;
}