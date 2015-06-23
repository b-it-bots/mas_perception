#ifndef BACKGROUNDCHANGEDETECTION_H_
#define BACKGROUNDCHANGEDETECTION_H_

#include <opencv2/opencv.hpp>

class BackgroundChangeDetection
{
    public:
        BackgroundChangeDetection();
        virtual ~BackgroundChangeDetection();
        bool detectBackgroundChange(const cv::Mat &current_frame, cv::Mat &debug_image);
        void updateDynamicVariables(bool debug_mode, double background_change_threshold, double background_learning_rate);

    private:
        cv::Ptr<cv::BackgroundSubtractor> pMOG;
        bool is_debug_mode_;
        double background_change_threshold_;
        double background_learning_rate_;
};

#endif /* BACKGROUNDCHANGEDETECTION_H_ */