/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */
#ifndef MAS_PERCEPTION_LIBS_IMAGE_BOUNDING_BOX_H_
#define MAS_PERCEPTION_LIBS_IMAGE_BOUNDING_BOX_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <mcr_perception_msgs/ImageList.h>

namespace mas_perception_libs
{

class ImageBoundingBox
{
public:
    ImageBoundingBox(const sensor_msgs::ImageConstPtr &, const sensor_msgs::CameraInfoConstPtr &,
                     const mcr_perception_msgs::BoundingBoxList::ConstPtr &);
    ~ImageBoundingBox();

    const mcr_perception_msgs::ImageList& cropped_image_list() const { return mCroppedImageList; }
    const std::vector<std::vector<cv::Point2f>>& box_vertices_vector() const { return mBoxVerticesVector; }

private:
    cv::Mat mImage;
    cv_bridge::CvImagePtr mImagePtr;
    image_geometry::PinholeCameraModel mCameraModel;
    tf::TransformListener mTfListener;

    mcr_perception_msgs::ImageList mCroppedImageList;
    std::vector<std::vector<cv::Point2f>> mBoxVerticesVector;
};

cv::Mat
cropImage(cv::Mat &image, std::vector<cv::Point2f> &vertices);

}  // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_IMAGE_BOUNDING_BOX_H_
