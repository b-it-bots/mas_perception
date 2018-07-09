/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <map>
#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <mas_perception_libs/impl/pyboostcvconverter.hpp>
#include <mas_perception_libs/bounding_box_2d.h>

namespace mas_perception_libs
{

void
drawLabeledBoxes(cv::Mat &pImage, std::vector<std::string> pLabels, std::vector<cv::Scalar> pColors,
                 std::vector<std::map<BoundingBox2DKey, double>> pBoxCoordsVect, int pThickness, double pFontScale)
{
    if (pLabels.size() != pColors.size() || pColors.size() != pBoxCoordsVect.size())
        throw std::invalid_argument("labels, colors and box coordinates vectors do not have the same size");

    for (size_t i = 0; i < pLabels.size(); i++)
    {
        // fit box to within image boundaries
        int minX = static_cast<int>(pBoxCoordsVect[i][BoundingBox2DKey::X_MIN]);
        if (minX < 0) minX = 0;

        int minY = static_cast<int>(pBoxCoordsVect[i][BoundingBox2DKey::Y_MIN]);
        if (minY < 0) minY = 0;

        int maxX = static_cast<int>(pBoxCoordsVect[i][BoundingBox2DKey::X_MAX]);
        if (maxX > pImage.cols) maxX = pImage.cols;

        int maxY = static_cast<int>(pBoxCoordsVect[i][BoundingBox2DKey::Y_MAX]);
        if (maxY > pImage.rows) maxY = pImage.rows;

        // fit label into within image boundaries
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(pLabels[i], cv::FONT_HERSHEY_PLAIN, pFontScale, pThickness, &baseline);
        if (minY < textSize.height) minY = textSize.height;

        // draw box
        cv::Point boxTopLeft(minX, minY);
        cv::Point boxBottomRight(maxX, maxY);
        cv::rectangle(pImage, boxTopLeft, boxBottomRight, pColors[i], pThickness);

        // draw label with background
        cv::Point textTopLeft = boxTopLeft - cv::Point(0, textSize.height);
        cv::Point textBottomRight = boxTopLeft + cv::Point(textSize.width, baseline);
        cv::rectangle(pImage, textTopLeft, textBottomRight, pColors[i], CV_FILLED);
        cv::putText(pImage, pLabels[i], boxTopLeft, cv::FONT_HERSHEY_PLAIN, pFontScale, CV_RGB(255, 255, 555), 1);
    }
}

}   // namespace mas_perception_libs
