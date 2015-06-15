#include <mcr_perception_selectors/object_selector.h>
#include <mcr_perception_msgs/Object.h>

#include <geometry_msgs/PoseStamped.h>


ObjectSelector::ObjectSelector() : nh_("~"), object_name_received_(false), object_list_received_(false)
{
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    pub_object_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("output/object_pose", 1);
    sub_object_list_ = nh_.subscribe("input/object_list", 1, &ObjectSelector::objectListCallback, this);
    sub_event_in_ = nh_.subscribe("event_in", 1, &ObjectSelector::eventCallback, this);
    sub_object_name_ = nh_.subscribe("input/object_name", 1, &ObjectSelector::objectNameCallback, this);
}

ObjectSelector::~ObjectSelector()
{
    pub_event_out_.shutdown();
    pub_object_pose_.shutdown();
    sub_object_list_.shutdown();
    sub_event_in_.shutdown();
    sub_object_name_.shutdown();
}

void ObjectSelector::objectNameCallback(const std_msgs::String::Ptr &msg)
{
    object_name_ = *msg;
    object_name_received_ = true;
}

void ObjectSelector::objectListCallback(const mcr_perception_msgs::ObjectList::Ptr &msg)
{
    object_list_ = msg;
    object_list_received_ = true;
}

void ObjectSelector::eventCallback(const std_msgs::String::Ptr &msg)
{
    event_in_ = *msg;
    event_in_received_ = true;
}
void ObjectSelector::update()
{
    if (!object_name_received_) {
        return;
    }

    object_name_received_ = false;

    std_msgs::String event_out;

    // no list received
    if (!object_list_received_) {
        event_out.data = "e_no_object_list";
        pub_event_out_.publish(event_out);
        return;
    }

    // list is currently empty (i.e. all objects have been selected previously)
    if (object_list_->objects.empty()) {
        event_out.data = "e_no_objects";
        pub_event_out_.publish(event_out);
        return;
    }

    std::vector<mcr_perception_msgs::Object>::iterator iter;

    for (iter = object_list_->objects.begin(); iter != object_list_->objects.end(); ++iter) {
        // selected object is erased from list
        if (object_name_.data == iter->name) {
            event_out.data = "e_selected";
            pub_object_pose_.publish(iter->pose);
            pub_event_out_.publish(event_out);
            object_list_->objects.erase(iter);
            return;
        }
    }

    // the desired object is not available
    event_out.data = "e_not_found";
    pub_event_out_.publish(event_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_selector");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    ObjectSelector object_selector;

    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO("node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok()) {
        object_selector.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
