/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#ifndef MAS_PERCEPTION_LIBS_BOUNDING_BOX_WRAPPER_H
#define MAS_PERCEPTION_LIBS_BOUNDING_BOX_WRAPPER_H

#include <string>
#include <boost/python.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mas_perception_libs/bounding_box.h>

namespace mas_perception_libs
{

class BoundingBoxWrapper
{
public:
    BoundingBoxWrapper(const std::string&, const boost::python::list&);
    ~BoundingBoxWrapper();

    std::string getPose();
    std::string getRosMsg();

private:
    BoundingBox mBox;
    geometry_msgs::PoseStamped mPose;

    void calculatePose(const std_msgs::Header&);
};

}  // namespace mas_perception_libs
#endif  // MAS_PERCEPTION_LIBS_BOUNDING_BOX_WRAPPER_H
