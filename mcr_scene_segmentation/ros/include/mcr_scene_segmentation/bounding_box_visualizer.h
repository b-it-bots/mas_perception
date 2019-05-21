/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_BOUNDING_BOX_VISUALIZER_H
#define MCR_SCENE_SEGMENTATION_BOUNDING_BOX_VISUALIZER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <mas_perception_msgs/BoundingBox.h>
#include <mas_perception_libs/color.h>

using mas_perception_libs::Color;

namespace mcr
{

namespace visualization
{

class BoundingBoxVisualizer
{
public:
    BoundingBoxVisualizer(ros::NodeHandle *nh, const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    BoundingBoxVisualizer(const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    void publish(const mas_perception_msgs::BoundingBox& box, const std::string& frame_id);

    void publish(const std::vector<mas_perception_msgs::BoundingBox>& boxes, const std::string& frame_id);

    int getNumSubscribers();

private:
    ros::Publisher marker_publisher_;

    const Color color_;
    bool check_subscribers_;
};

}  // namespace visualization

}  // namespace mcr
#include "impl/bounding_box_visualizer.hpp"

#endif  // MCR_SCENE_SEGMENTATION_BOUNDING_BOX_VISUALIZER_H
