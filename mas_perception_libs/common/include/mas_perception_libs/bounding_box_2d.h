#include <utility>

/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 */
#ifndef MAS_PERCEPTION_LIBS_BOUNDING_BOX_2D_H
#define MAS_PERCEPTION_LIBS_BOUNDING_BOX_2D_H

#include <string>
#include <vector>
#include <opencv/cv.h>

namespace mas_perception_libs
{

struct BoundingBox2D
{
    /*!
     * @brief struct for representing a rectangle region in RGB images
     */
    int mX;
    int mY;
    int mWidth;
    int mHeight;
    std::string mLabel;
    cv::Scalar mColor;

    BoundingBox2D(const std::string &pLabel, const cv::Scalar &pColor, int pX, int pY, int pWidth, int pHeight)
            : mX(pX), mY(pY), mWidth(pWidth), mHeight(pHeight), mLabel(pLabel), mColor(pColor)
    { }

    BoundingBox2D(const std::string &pLabel, int pX, int pY, int pWidth, int pHeight)
            : BoundingBox2D(pLabel, CV_RGB(0, 0, 255), pX, pY, pWidth, pHeight)
    { }

    BoundingBox2D(int pX, int pY, int pWidth, int pHeight)
            : BoundingBox2D("", pX, pY, pWidth, pHeight)
    { }

    BoundingBox2D() : BoundingBox2D(0, 0, 0, 0)
    { }

    /*! @brief update box geometry using a cv::Rect object */
    void updateBox(const cv::Rect &);

    /*! @brief get box geometry as a cv::Rect object */
    cv::Rect getCvRect() { return cv::Rect(mX, mY, mWidth, mHeight); }
};

/*!
 * @brief draw boxes on a CV image using BoundingBox2D objects
 * @param pImage
 * @param pBoundingBoxes: vector of bounding boxes to be drawn, elements may be modified to fit image size
 * @param pThickness: line thickness
 * @param pFontScale
 */
void
drawLabeledBoxes(cv::Mat &pImage, std::vector<BoundingBox2D> pBoundingBoxes,
                 int pThickness = 2, double pFontScale = 1.0);

/*!
 * @brief adjust a BoundingBox2D object to image size
 * @param pImageSize: image dimensions
 * @param pBox: BoundingBox2D object to be adjusted
 * @param pSizeOffset: offset to increase box dimensions
 */
void
fitBoxToImage(const cv::Size &pImageSize, BoundingBox2D &pBox, int pSizeOffset = 0);

/*!
 * @brief adjust a cv::Rect object to image size
 * @return adjusted box as cv::Rect object
 */
cv::Rect
fitBoxToImage(const cv::Size &pImageSize, cv::Rect pBox, int pSizeOffset = 0);

/*!
 * @brief crop a CV image using a set of 2D vertices
 * @param pImage
 * @param pVertices: vector of vertices to specify the cropping region
 * @param pOffset: offset for expanding the bounding box's dimensions
 * @param pCopy: The CV image will be copied during cropping. By default OpenCV will only shift the image pointers
 * @return cropped CV image
 */
cv::Mat
cropImage(cv::Mat &pImage, const std::vector<cv::Point2f> &pVertices, int pOffset = 0, bool pCopy = true);

/*!
 * @brief crop a CV image using a BoundingBox2D object
 * @param pBox: bounding box region to crop, may be adjusted to fit image size
 */
cv::Mat
cropImage(cv::Mat &pImage, BoundingBox2D &pBox, int pOffset = 0, bool pCopy = true);

/*!
 * @brief crop a CV image using a cv::Rect object
 * @param pRoiRect: region-of-interest rectangle, maybe adjusted to fit image size
 */
cv::Mat
cropImage(cv::Mat &pImage, cv::Rect &pRoiRect, int pOffset = 0, bool pCopy = true);

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_BOUNDING_BOX_2D_H
