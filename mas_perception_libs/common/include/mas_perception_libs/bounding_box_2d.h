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
#include <Python.h>
#include <opencv/cv.h>
#include <boost/python.hpp>

namespace bp = boost::python;

namespace mas_perception_libs
{
    enum BoundingBox2DKey
    {
        LABEL = 0,
        COLOR,
        X_MIN,
        Y_MIN,
        X_MAX,
        Y_MAX
    };

    void
    drawLabeledBoxes(cv::Mat &pImage, std::vector<std::string> pLabels, std::vector<cv::Scalar> pColors,
                     std::vector<std::map<BoundingBox2DKey, double>> pBoxCoordsVect, int pThickness, double pFontScale);

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_BOUNDING_BOX_2D_H
