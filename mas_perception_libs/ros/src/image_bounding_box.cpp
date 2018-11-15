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

        cv::Mat croppedImage = cropImage(mImagePtr->image, imageVertices);

        cv_bridge::CvImage image_msg;
        image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        image_msg.image = croppedImage;
        image_msg.header = pImageMsg.header;

        mCroppedImageList.images.push_back(*image_msg.toImageMsg());
        mBoxVerticesVector.push_back(imageVertices);
    }
}

ImageBoundingBox::~ImageBoundingBox() = default;

cv::Mat
cropImage(cv::Mat &image, std::vector<cv::Point2f> &vertices)
{
    cv::Rect roi_rectangle = cv::boundingRect(cv::Mat(vertices));
    // expand rectangle a bit
    // (move top left by 5x5 pixels, and increase size by 10 x 10)
    roi_rectangle -= cv::Point(5, 5);
    roi_rectangle += cv::Size(10, 10);
    cv::Rect image_rect(0, 0, image.cols, image.rows);

    // check if roi is contained within image
    if (!((roi_rectangle & image_rect) == roi_rectangle))
    {
        if (roi_rectangle.x < 0)
        {
            roi_rectangle.x = 0;
        }
        if (roi_rectangle.y < 0)
        {
            roi_rectangle.y = 0;
        }
        if (roi_rectangle.x + roi_rectangle.width >= image.cols)
        {
            roi_rectangle.width = image.cols - roi_rectangle.x - 1;
        }
        if (roi_rectangle.y + roi_rectangle.height >= image.rows)
        {
            roi_rectangle.height = image.cols - roi_rectangle.y - 1;
        }
    }

    cv::Mat cropped_image(image, roi_rectangle);

    return cropped_image;
}

}  // namespace mas_perception_libs
