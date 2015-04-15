#include <mcr_edit_image/edit_image_node.h>

EditImageNode::EditImageNode(ros::NodeHandle &nh) : node_handler_(nh), image_transporter_(nh)
{
    dynamic_reconfig_server_.setCallback(boost::bind(&EditImageNode::dynamicReconfigCallback, this, _1, _2));
    event_sub_ = node_handler_.subscribe("event_in", 1, &EditImageNode::eventCallback, this);
    image_pub_ = image_transporter_.advertise("edited_image", 1);
    image_sub_ = image_transporter_.subscribe("input_image", 1, &EditImageNode::imageCallback, this);
    run_state_ = INIT;
    start_edit_image_ = false;
    image_sub_status_ = false;  
}

EditImageNode::~EditImageNode()
{
    event_sub_.shutdown();
    image_pub_.shutdown();
    image_sub_.shutdown();
}

void EditImageNode::dynamicReconfigCallback(mcr_edit_image::EditImageConfig &config, uint32_t level)
{
    is_rotation_ = config.rotate_image;
    is_crop_ = config.crop_image;
    crop_factor_top_ = config.crop_factor_top;
    crop_factor_bottom_ = config.crop_factor_bottom;
    crop_factor_left_ = config.crop_factor_left;
    crop_factor_right_ = config.crop_factor_right;
}

void EditImageNode::eventCallback(const std_msgs::String &event_command)
{
    if (event_command.data == "e_start") {
        start_edit_image_ = true;
        ROS_INFO("2D Edit Image ENABLED");
    } else if (event_command.data == "e_stop") {
        start_edit_image_ = false;
        ROS_INFO("2D Edit Image DISABLED");
    }
}

void EditImageNode::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    image_message_ = img_msg;
    image_sub_status_ = true;
}

void EditImageNode::states()
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

void EditImageNode::initState()
{
    if (image_sub_status_) {
        run_state_ = IDLE;
        image_sub_status_ = false;
    }
}

void EditImageNode::idleState()
{
    if (start_edit_image_) {
        run_state_ = RUNNING;
    } else {
        run_state_ = INIT;
    }
}

void EditImageNode::runState()
{
    editImage();
    run_state_ = INIT;
}

void EditImageNode::editImage()
{ 

    cv_bridge::CvImagePtr cv_img_ptr;
    
    try {
        cv_img_ptr = cv_bridge::toCvCopy(image_message_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_message_->encoding.c_str());
    }

    if(is_crop_){
        Size cv_img_size = cv_img_ptr->image.size();
        int height = cv_img_size.height;
        int width = cv_img_size.width;
        Rect roi(width*crop_factor_left_, height*crop_factor_top_, (width-width*crop_factor_left_)-width*crop_factor_right_, (height-height*crop_factor_top_)-height*crop_factor_bottom_);
        cv_img_ptr->image = cv_img_ptr->image(roi);
    }

    if(is_rotation_){
        flip(cv_img_ptr->image, cv_img_ptr->image, -1);
    }

    image_pub_.publish(cv_img_ptr->toImageMsg());

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "edit_image");
    ros::NodeHandle nh("~");
    ROS_INFO("Edit Image Node Initialised");
    EditImageNode ei(nh);
    while (ros::ok()) {
        ei.states();
        ros::spinOnce();
    }
    return 0;
}