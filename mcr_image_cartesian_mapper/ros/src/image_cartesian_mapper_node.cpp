#include <mcr_image_cartesian_mapper/image_cartesian_mapper_node.h>

ImageCartesianMapperNode::ImageCartesianMapperNode(ros::NodeHandle &nh) : node_handler_(nh), image_transporter_(nh)
{
    event_sub_ = node_handler_.subscribe("event_in", 1, &ImageCartesianMapperNode::eventCallback, this);
    pose_sub_ = node_handler_.subscribe("pose", 1, &ImageCartesianMapperNode::poseCallback, this);
    image_sub_ = image_transporter_.subscribe("image", 1, &ImageCartesianMapperNode::imageCallback, this);
    cartesian_pub_ = node_handler_.advertise<geometry_msgs::PoseStamped>("cartesian_pose", 1);
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    run_state_ = INIT;
    start_cartesian_mapper_ = false;
    pose_sub_status_ = false;
    image_sub_status_ = false; 
}

ImageCartesianMapperNode::~ImageCartesianMapperNode()
{
    event_sub_.shutdown();
    pose_sub_.shutdown();
    image_sub_.shutdown();
    cartesian_pub_.shutdown();
    event_pub_.shutdown();  
}



void ImageCartesianMapperNode::eventCallback(const std_msgs::String &event_command)
{
    if (event_command.data == "e_start") {
        start_cartesian_mapper_ = true;
        ROS_INFO("Image to Cartesian Mapper ENABLED");
    } else if (event_command.data == "e_stop") {
        start_cartesian_mapper_ = false;
        ROS_INFO("Image to Cartesian Mapper DISABLED");
    }
}

void ImageCartesianMapperNode::poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose)
{
    pose_2d_ = *pose;
    pose_sub_status_ = true;
}

void ImageCartesianMapperNode::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    image_message_ = image;
    image_sub_status_ = true;

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
    if (pose_sub_status_ && image_sub_status_) {
        run_state_ = IDLE;
        pose_sub_status_ = false;
        image_sub_status_ = false;
    }
}

void ImageCartesianMapperNode::idleState()
{
    if (start_cartesian_mapper_) {
        run_state_ = RUNNING;
    } else {
        run_state_ = INIT;
    }
}

void ImageCartesianMapperNode::runState()
{
    imagetoCartesianMapper();
    run_state_ = INIT;
}

void ImageCartesianMapperNode::imagetoCartesianMapper()
{

    cv_bridge::CvImagePtr cv_img_ptr;
    
    try {
        cv_img_ptr = cv_bridge::toCvCopy(image_message_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_message_->encoding.c_str());
        return;
    }

    Size cv_img_size = cv_img_ptr->image.size();
    center_x_ = cv_img_size.height/2;
    center_y_ = cv_img_size.width/2;


    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;

    point.x = pose_2d_.x - center_x_;
    point.y = pose_2d_.y - center_y_;
    quaternion = tf::createQuaternionMsgFromYaw(pose_2d_.theta);
    pose.position = point;
    pose.orientation = quaternion;

    cartesian_pose_.header.stamp = ros::Time::now();
    cartesian_pose_.header.frame_id = "/tower_cam3d_rgb_optical_frame";
    cartesian_pose_.pose = pose;

    cartesian_pub_.publish(cartesian_pose_);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_cartesian_mapper");
    ros::NodeHandle nh("~");
    ROS_INFO("Image to Cartesian Mapper Node Initialised");
    ImageCartesianMapperNode icm(nh);
    while (ros::ok()) {
        icm.states();
        ros::spinOnce();
    }
    return 0;
}