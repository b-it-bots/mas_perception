/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <vector>
#include <string>
#include <boost/python.hpp>
#include <numpy/arrayobject.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <mas_perception_libs/use_numpy.h>
#include <mas_perception_libs/impl/pyboostcvconverter.hpp>
#include <mas_perception_libs/impl/ros_message_serialization.hpp>
#include <mas_perception_libs/bounding_box_wrapper.h>
#include <mas_perception_libs/image_bounding_box.h>
#include <mas_perception_libs/bounding_box_2d.h>
#include <mas_perception_libs/point_cloud_utils.h>

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
            throw std::invalid_argument("box geometry is not a tuple containing 4 numerics");
        }
        mX = static_cast<int>(bp::extract<double>(pBox[0]));
        mY = static_cast<int>(bp::extract<double>(pBox[1]));
        mWidth = static_cast<int>(bp::extract<double>(pBox[2]));
        mHeight = static_cast<int>(bp::extract<double>(pBox[3]));
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

BoundingBox2DWrapper
fitBoxToImageWrapper(bp::tuple pImageSizeTuple, BoundingBox2DWrapper pBox, int pOffset)
{
    if (bp::len(pImageSizeTuple) != 2)
    {
        throw std::invalid_argument("image size is not a tuple containing 2 numerics");
    }
    int width = static_cast<int>(bp::extract<double>(pImageSizeTuple[0]));
    int height = static_cast<int>(bp::extract<double>(pImageSizeTuple[1]));
    cv::Size imageSize(width, height);
    cv::Rect adjustedBox = fitBoxToImage(imageSize, pBox.getCvRect(), pOffset);
    pBox.updateBox(adjustedBox);
    return pBox;
}

PyObject *
cropImageWrapper(PyObject * pNdarrayImage, BoundingBox2DWrapper pBox, int pOffset)
{
    cv::Mat image = pbcvt::fromNDArrayToMat(pNdarrayImage);
    cv::Mat croppedImage = cropImage(image, pBox, pOffset);
    PyObject *ret = pbcvt::fromMatToNDArray(croppedImage);
    return ret;
}

PyObject *
cloudMsgToCvImageWrapper(std::string pSerialCloud)
{
    // unserialize cloud message
    sensor_msgs::PointCloud2 cloudMsg = from_python<sensor_msgs::PointCloud2>(std::move(pSerialCloud));

    // get cv::Mat object and convert to ndarray object
    cv::Mat image = cloudMsgToCvImage(cloudMsg);
    PyObject *ret = pbcvt::fromMatToNDArray(image);

    return ret;
}

std::string
cloudMsgToImageMsgWrapper(std::string pSerialCloud)
{
    // unserialize cloud message
    sensor_msgs::PointCloud2 cloudMsg = from_python<sensor_msgs::PointCloud2>(std::move(pSerialCloud));

    // check for organized cloud and extract image message
    if (cloudMsg.height <= 1)
    {
        throw std::invalid_argument("Input point cloud is not organized!");
    }
    sensor_msgs::Image imageMsg;
    pcl::toROSMsg(cloudMsg, imageMsg);

    return to_python(imageMsg);
}

std::string
cropOrganizedCloudMsgWrapper(std::string pSerialCloud, BoundingBox2DWrapper pBox)
{
    // unserialize cloud message
    sensor_msgs::PointCloud2 cloudMsg = from_python<sensor_msgs::PointCloud2>(std::move(pSerialCloud));

    sensor_msgs::PointCloud2 croppedCloudMsg;
    cropOrganizedCloudMsg(cloudMsg, pBox, croppedCloudMsg);

    return to_python(croppedCloudMsg);
}

PyObject *
cropCloudMsgToXYZWrapper(const std::string &pSerialCloud, BoundingBox2DWrapper pBox)
{
    // unserialize cloud message
    sensor_msgs::PointCloud2 cloudMsg = from_python<sensor_msgs::PointCloud2>(pSerialCloud);

    cv::Mat coords = cropCloudMsgToXYZ(cloudMsg, pBox);

    PyObject *coordArray = pbcvt::fromMatToNDArray(coords);

    return coordArray;
}

std::string
transformPointCloudWrapper(const std::string &pSerialCloud, PyObject * pTfMatrix)
{
    // convert tf matrix from cv::Mat to Eigen::Matrix
    cv::Mat tfMatrix = pbcvt::fromNDArrayToMat(pTfMatrix);
    if (tfMatrix.rows != 4 || tfMatrix.cols != 4)
        throw std::runtime_error("transformation is not a 4x4 matrix");
    Eigen::Matrix4f eigenTfMatrix;
    cv::cv2eigen(tfMatrix, eigenTfMatrix);

    // unserialize cloud message
    sensor_msgs::PointCloud2 cloudMsg = from_python<sensor_msgs::PointCloud2>(pSerialCloud);

    // transform using tf matrix
    sensor_msgs::PointCloud2 transformedCloud;
    pcl_ros::transformPointCloud(eigenTfMatrix, cloudMsg, transformedCloud);

    // serialize and return trasnformed cloud
    return to_python(transformedCloud);
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

    bp::def("_fit_box_to_image", mas_perception_libs::fitBoxToImageWrapper);

    bp::def("_crop_image", mas_perception_libs::cropImageWrapper);

    bp::def("_cloud_msg_to_cv_image", mas_perception_libs::cloudMsgToCvImageWrapper);

    bp::def("_cloud_msg_to_image_msg", mas_perception_libs::cloudMsgToImageMsgWrapper);

    bp::def("_crop_organized_cloud_msg", mas_perception_libs::cropOrganizedCloudMsgWrapper);

    bp::def("_crop_cloud_to_xyz", mas_perception_libs::cropCloudMsgToXYZWrapper);

    bp::def("_transform_point_cloud", mas_perception_libs::transformPointCloudWrapper);
}
