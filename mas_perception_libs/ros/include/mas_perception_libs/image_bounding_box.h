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
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <mcr_perception_msgs/ImageList.h>
#include <mas_perception_libs/bounding_box_2d.h>

namespace mas_perception_libs
{

class ImageBoundingBox
{
public:
    ImageBoundingBox(const sensor_msgs::Image&, const sensor_msgs::CameraInfo&,
                     const mcr_perception_msgs::BoundingBoxList&);
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

/*!
 * @brief Draw boxes on a sensor_msgs/Image message using BoundingBox2D objects. Call drawLabeledBoxes() defined in
 *        bounding_box_2d.h in the background.
 * @param pImage: image message to draw on, will be copied to a CV image before boxes are drawn on top
 */
sensor_msgs::ImagePtr
drawLabeledBoxesImgMsg(const sensor_msgs::Image& pImage, std::vector<BoundingBox2D>,
                       int pThickness = 2, double pFontScale = 1.0);

}  // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_IMAGE_BOUNDING_BOX_H_
