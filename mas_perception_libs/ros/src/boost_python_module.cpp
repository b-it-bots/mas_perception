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

struct BoundingBox2DWrapper : BoundingBox2D
{
    // TODO(minhnh): expose color

    BoundingBox2DWrapper(std::string label, bp::tuple color, bp::tuple pBox) : BoundingBox2DWrapper(label, pBox)
    {
        // extract color
        if (bp::len(color) != 3)
        {
            throw std::invalid_argument("color is not a 3-tuple containing integers");
        }
        mColor = CV_RGB(static_cast<int>(bp::extract<double>(color[0])),
                        static_cast<int>(bp::extract<double>(color[1])),
                        static_cast<int>(bp::extract<double>(color[2])));
    }

    BoundingBox2DWrapper(std::string label, bp::tuple pBox) : BoundingBox2DWrapper(std::move(pBox))
    {
        mLabel = std::move(label);
    }

    explicit BoundingBox2DWrapper(bp::tuple pBox) : BoundingBox2D()
    {
        // extract box geometry
        if (bp::len(pBox) != 4)
        {
            throw std::invalid_argument("expect box geometry to be list containing 4 doubles, ");
        }
        mX = static_cast<int>(bp::extract<double>(pBox[0]));
        mY = static_cast<int>(bp::extract<double>(pBox[1]));
        mWidth = static_cast<int>(bp::extract<double>(pBox[2]));
        mHeight = static_cast<int>(bp::extract<double>(pBox[3]));
        mCvRect = cv::Rect(mX, mY, mWidth, mHeight);
    }
};

PyObject *
drawLabeledBoxesWrapper(PyObject * pNdarrayImage, bp::list pBoxList, int pThickness, double pFontScale)
{
    cv::Mat image = pbcvt::fromNDArrayToMat(pNdarrayImage);
    std::vector<BoundingBox2D> boundingBoxes;
    for (int i = 0; i < bp::len(pBoxList); i++)
    {
        BoundingBox2DWrapper boundingBox = bp::extract<BoundingBox2DWrapper>(pBoxList[i]);
        boundingBoxes.push_back(boundingBox);
    }

    // draw boxes
    drawLabeledBoxes(image, boundingBoxes, pThickness, pFontScale);

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
    using mas_perception_libs::BoundingBox2DWrapper;

    bp::class_<BoundingBoxWrapper>("BoundingBoxWrapper", bp::init<std::string, bp::list&>())
            .def("get_pose", &BoundingBoxWrapper::getPose)
            .def("get_ros_message", &BoundingBoxWrapper::getRosMsg);

    bp::def("get_crops_and_bounding_boxes_wrapper", mas_perception_libs::getCropsAndBoundingBoxes);

    bp::class_<BoundingBox2DWrapper>("BoundingBox2DWrapper", bp::init<bp::tuple&>())
            .def(bp::init<std::string, bp::tuple&>())
            .def(bp::init<std::string, bp::tuple&, bp::tuple&>())
            .def_readwrite("x", &BoundingBox2DWrapper::mX)
            .def_readwrite("y", &BoundingBox2DWrapper::mY)
            .def_readwrite("width", &BoundingBox2DWrapper::mWidth)
            .def_readwrite("height", &BoundingBox2DWrapper::mHeight)
            .def_readwrite("label", &BoundingBox2DWrapper::mLabel);

    bp::def("_draw_labeled_boxes", mas_perception_libs::drawLabeledBoxesWrapper);
}
