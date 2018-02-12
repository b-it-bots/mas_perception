#ifndef BACKGROUNDCHANGEDETECTION_H_
#define BACKGROUNDCHANGEDETECTION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include "opencv2/bgsegm.hpp"

using namespace cv;

class BackgroundChangeDetection
{
public:
    BackgroundChangeDetection();
    virtual ~BackgroundChangeDetection();
    bool detectBackgroundChange(const cv::Mat &current_frame, cv::Mat &debug_image);
    void initializeBackgroundModel(const cv::Mat &current_frame);
    void updateDynamicVariables(bool debug_mode, double background_change_threshold, double background_learning_rate);

private:
    Ptr<BackgroundSubtractor> bsmog_;
    bool is_debug_mode_;
    double background_change_threshold_;
    double background_learning_rate_;
};

#endif /* BACKGROUNDCHANGEDETECTION_H_ */
