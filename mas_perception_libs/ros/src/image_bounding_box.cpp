/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen, Santosh Thoduka
 *
 */
#include <vector>
#include <string>
#include <stdexcept>

#include <tf/transform_listener.h>
#include <mas_perception_libs/image_bounding_box.h>
#include <mas_perception_libs/bounding_box_2d.h>

namespace mas_perception_libs
{

ImageBoundingBox::ImageBoundingBox(const sensor_msgs::Image &pImageMsg,
                                   const sensor_msgs::CameraInfo &pCameraInfo,
                                   const mcr_perception_msgs::BoundingBoxList &pBoundingBoxList)
{
    if (pBoundingBoxList.bounding_boxes.empty())
    {
        throw std::invalid_argument("bounding box list is empty");
    }

    // get camera model
    mCameraModel.fromCameraInfo(pCameraInfo);

    // convert to CV image
    try
    {
        mImagePtr = cv_bridge::toCvCopy(pImageMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &ex)
    {
        std::ostringstream message;
        message << "can't convert ROS image message to CV image: " << ex.what();
        throw std::runtime_error(message.str());
    }

    // wait for transform
    std::string object_frame_id = pBoundingBoxList.header.frame_id;
    std::string image_frame_id = pImageMsg.header.frame_id;
    try
    {
        mTfListener.waitForTransform(object_frame_id, image_frame_id,
                                    pImageMsg.header.stamp, ros::Duration(1.0));
    }
    catch (tf::TransformException &ex)
    {
        std::ostringstream message;
        message << "caught exception waiting for transform: " << ex.what();
        throw std::runtime_error(message.str());
    }

    for (const auto &boundingBox : pBoundingBoxList.bounding_boxes)
    {
        // transform vertices to image frame
        std::vector<cv::Point2f> imageVertices;
        for (const auto &vertice : boundingBox.vertices)
        {
            geometry_msgs::PointStamped point;
            geometry_msgs::PointStamped transformedPoint;
            point.point = vertice;
            point.header.frame_id = object_frame_id;
            point.header.stamp = pImageMsg.header.stamp;
            try
            {
                mTfListener.transformPoint(image_frame_id, point, transformedPoint);
            }
            catch (tf::TransformException &ex)
            {
                std::ostringstream message;
                message << "failed to transform point from frame '" << object_frame_id
                        << "' to frame '" << image_frame_id << "': " << ex.what();
                throw std::runtime_error(message.str());
            }

            cv::Point3d point_xyz(transformedPoint.point.x, transformedPoint.point.y, transformedPoint.point.z);
            cv::Point2f uv;
            uv = mCameraModel.project3dToPixel(point_xyz);
            imageVertices.push_back(uv);
        }

        cv::Mat croppedImage = cropImage(mImagePtr->image, imageVertices, 5);

        cv_bridge::CvImage image_msg;
        image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        image_msg.image = croppedImage;
        image_msg.header = pImageMsg.header;

        mCroppedImageList.images.push_back(*image_msg.toImageMsg());
        mBoxVerticesVector.push_back(imageVertices);
    }
}

ImageBoundingBox::~ImageBoundingBox() = default;

}  // namespace mas_perception_libs
