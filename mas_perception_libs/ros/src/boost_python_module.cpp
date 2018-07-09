/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <map>
#include <vector>
#include <utility>
#include <string>
#include <boost/python.hpp>
#include <numpy/arrayobject.h>
#include <mas_perception_libs/use_numpy.h>
#include <mas_perception_libs/impl/pyboostcvconverter.hpp>
#include <mas_perception_libs/impl/ros_message_serialization.hpp>
#include <mas_perception_libs/bounding_box_wrapper.h>
#include <mas_perception_libs/image_bounding_box.h>
#include <mas_perception_libs/bounding_box_2d.h>

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
    const std::vector<std::vector<cv::Point2f>> &boxVerticesVect = mImgBoundingBox.box_vertices_vector();
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

PyObject *
drawLabeledBoxesWrapper(PyObject * pNdarrayImage, bp::list pBoxList, int pThickness, double pFontScale)
{
    cv::Mat image = pbcvt::fromNDArrayToMat(pNdarrayImage);
    std::vector<std::string> labels;
    std::vector<cv::Scalar> colors;
    std::vector<std::map<BoundingBox2DKey, double>> boxCoordsVect;
    for (int i = 0; i < bp::len(pBoxList); i++)
    {
        bp::dict boundingBox = bp::extract<bp::dict>(pBoxList[i]);

        // extract labels
        labels.push_back(bp::extract<std::string>(boundingBox[BoundingBox2DKey::LABEL]));

        // extract colors
        bp::tuple colorTuple = bp::extract<bp::tuple>(boundingBox[BoundingBox2DKey::COLOR]);
        if (bp::len(colorTuple) != 3)
        {
            throw std::invalid_argument("expect colors to be 3-tuples containing integers");
        }
        cv::Scalar color = CV_RGB(bp::extract<int>(colorTuple[0]), bp::extract<int>(colorTuple[1]),
                                  bp::extract<int>(colorTuple[2]));
        colors.push_back(color);

        // extract coordinates
        std::map<BoundingBox2DKey, double> boxCoords;
        int minX = bp::extract<double>(boundingBox[BoundingBox2DKey::X_MIN]);
        int minY = bp::extract<double>(boundingBox[BoundingBox2DKey::Y_MIN]);
        int maxX = bp::extract<double>(boundingBox[BoundingBox2DKey::X_MAX]);
        int maxY = bp::extract<double>(boundingBox[BoundingBox2DKey::Y_MAX]);
        boxCoords.insert(std::pair<BoundingBox2DKey, double>(BoundingBox2DKey::X_MIN, minX));
        boxCoords.insert(std::pair<BoundingBox2DKey, double>(BoundingBox2DKey::Y_MIN, minY));
        boxCoords.insert(std::pair<BoundingBox2DKey, double>(BoundingBox2DKey::X_MAX, maxX));
        boxCoords.insert(std::pair<BoundingBox2DKey, double>(BoundingBox2DKey::Y_MAX, maxY));
        boxCoordsVect.push_back(boxCoords);
    }

    // draw boxes
    drawLabeledBoxes(image, labels, colors, boxCoordsVect, pThickness, pFontScale);

    // convert to Python object and return
    PyObject *ret = pbcvt::fromMatToNDArray(image);
    return ret;
}

}  // namespace mas_perception_libs

BOOST_PYTHON_MODULE(_cpp_wrapper)
{
    // initialize converters
    bp::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
    pbcvt::matFromNDArrayBoostConverter();

    using mas_perception_libs::BoundingBoxWrapper;

    bp::class_<BoundingBoxWrapper>("BoundingBoxWrapper", bp::init<std::string, bp::list&>())
            .def("get_pose", &BoundingBoxWrapper::getPose)
            .def("get_ros_message", &BoundingBoxWrapper::getRosMsg);
    bp::def("get_crops_and_bounding_boxes_wrapper", mas_perception_libs::getCropsAndBoundingBoxes);
    bp::def("_draw_labeled_boxes", mas_perception_libs::drawLabeledBoxesWrapper);
    bp::enum_<mas_perception_libs::BoundingBox2DKey>("BoundingBox2DKey")
            .value("LABEL", mas_perception_libs::BoundingBox2DKey::LABEL)
            .value("COLOR", mas_perception_libs::BoundingBox2DKey::COLOR)
            .value("X_MIN", mas_perception_libs::BoundingBox2DKey::X_MIN)
            .value("Y_MIN", mas_perception_libs::BoundingBox2DKey::Y_MIN)
            .value("X_MAX", mas_perception_libs::BoundingBox2DKey::X_MAX)
            .value("Y_MAX", mas_perception_libs::BoundingBox2DKey::Y_MAX);
}
