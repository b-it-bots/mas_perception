/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <string>
#include <vector>
#include <boost/python.hpp>
#include <mas_perception_libs/impl/ros_message_serialization.hpp>
#include <mas_perception_libs/bounding_box_wrapper.h>
#include <mas_perception_libs/image_bounding_box.h>

namespace bp = boost::python;
using BoundingBox = mas_perception_libs::BoundingBox;

namespace mas_perception_libs
{

bp::tuple
getCropsAndBoundingBoxes(std::string pSerialImageMsg, std::string pSerialCameraInfo,
                         std::string pSerialBoundingBoxList)
{
    const sensor_msgs::Image &imageMsg = from_python<sensor_msgs::Image>(std::move(pSerialImageMsg));
    const sensor_msgs::CameraInfo &camInfo = from_python<sensor_msgs::CameraInfo>(std::move(pSerialCameraInfo));
    const mcr_perception_msgs::BoundingBoxList &boundingBoxList
            = from_python<mcr_perception_msgs::BoundingBoxList>(std::move(pSerialBoundingBoxList));
    ImageBoundingBox mImgBoundingBox(imageMsg, camInfo, boundingBoxList);

    // serialize image list
    const mcr_perception_msgs::ImageList &imageList = mImgBoundingBox.cropped_image_list();
    std::string serialImageList = to_python(imageList);

    // convert vector to bp::list
    const std::vector<std::vector<cv::Point2f>> boxVerticesVect = mImgBoundingBox.box_vertices_vector();
    bp::list boxVerticesList;
    for (const auto &boxVertices : boxVerticesVect)
    {
        bp::list boostBoxVertices;
        for (const auto &vertex : boxVertices)
        {
            boost::array<float, 2> boostVertex{};
            boostVertex[0] = vertex.x;
            boostVertex[1] = vertex.y;
            boostBoxVertices.append(boostVertex);
        }
        boxVerticesList.append(boostBoxVertices);
    }

    // return result tuple
    return bp::make_tuple<std::string, bp::list>(serialImageList, boxVerticesList);
}

}  // namespace mas_perception_libs

BOOST_PYTHON_MODULE(_cpp_wrapper)
{
    using mas_perception_libs::BoundingBoxWrapper;
    bp::class_<BoundingBoxWrapper>("BoundingBoxWrapper", bp::init<std::string, bp::list&>())
            .def("get_pose", &BoundingBoxWrapper::getPose)
            .def("get_ros_message", &BoundingBoxWrapper::getRosMsg);
    bp::def("get_crops_and_bounding_boxes_wrapper", mas_perception_libs::getCropsAndBoundingBoxes);
}
