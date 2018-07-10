/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#ifndef MAS_PERCEPTION_LIBS_BOUNDING_BOX_2D_H
#define MAS_PERCEPTION_LIBS_BOUNDING_BOX_2D_H

#include <map>
#include <string>
#include <vector>
#include <opencv/cv.h>

namespace mas_perception_libs
{

struct BoundingBox2D
{
    int mX;
    int mY;
    int mWidth;
    int mHeight;
    std::string mLabel;
    cv::Scalar mColor;

    BoundingBox2D(int pX, int pY, int pWidth, int pHeight)
        : mX(pX), mY(pY), mWidth(pWidth), mHeight(pHeight), mLabel(""), mColor(CV_RGB(0, 0, 255))
    { }

    BoundingBox2D() : BoundingBox2D(0, 0, 0, 0)
    { }

    void updateBox(cv::Rect);

    cv::Rect getCvRect() { return cv::Rect(mX, mY, mWidth, mHeight); }
};

void
drawLabeledBoxes(cv::Mat &pImage, std::vector<BoundingBox2D> pBoundingBoxes, int pThickness, double pFontScale);

cv::Rect
fitBoxToImage(cv::Size pImageSize, cv::Rect pBox, int pSizeOffset = 0);

cv::Mat
cropImage(cv::Mat &pImage, std::vector<cv::Point2f> &pVertices, int pOffset = 0, bool copy = true);

cv::Mat
cropImage(cv::Mat &pImage, BoundingBox2D &pBox, int pOffset = 0, bool copy = true);

cv::Mat
cropImage(cv::Mat &pImage, const cv::Rect &pRoiRect, int pOffset = 0, bool copy = true);

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_BOUNDING_BOX_2D_H
