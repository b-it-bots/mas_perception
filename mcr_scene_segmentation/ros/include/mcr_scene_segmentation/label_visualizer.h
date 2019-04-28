/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka, Sergey Alexandrov
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_LABEL_VISUALIZER_H
#define MCR_SCENE_SEGMENTATION_LABEL_VISUALIZER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <mas_perception_libs/color.h>

using mas_perception_libs::Color;

namespace mcr
{

namespace visualization
{

class LabelVisualizer
{
public:
    LabelVisualizer(const ros::NodeHandle &nh, const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    LabelVisualizer(const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    void publish(const std::vector<std::string> &labels, const geometry_msgs::PoseArray &poses);

    int getNumSubscribers();

private:
    ros::Publisher marker_publisher_;

    const Color color_;
    bool check_subscribers_;
};

}  // namespace visualization

}  // namespace mcr
#include "impl/label_visualizer.hpp"

#endif  // MCR_SCENE_SEGMENTATION_LABEL_VISUALIZER_H
