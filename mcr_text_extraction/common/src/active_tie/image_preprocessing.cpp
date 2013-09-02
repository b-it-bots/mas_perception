/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/image_preprocessing.h"



namespace atie {
  void TemporalAverage::setImages(vector<cv::Mat> images_) {
    init(images_[0]);
    for (int i= 0; i < images_.size(); i++) {
      add(images_[i]);
    }
    average();
  }
 
  void TemporalAverage::init(cv::Mat& m) {
    type= CV_32FC(m.channels());
    returnType= m.type();
    int nChannels= m.channels();
    if (averaged.cols != m.cols || m.rows != m.rows) {
      sum= cv::Mat(m.rows, m.cols, type);
      averaged= cv::Mat(m.rows, m.cols, m.type());
      averagedFloat= cv::Mat(m.rows, m.cols, type);
    }
    else {
      sum= cv::Scalar(0, 0, 0, 0);
    }    
    n= 0;
  }

  void TemporalAverage::add(cv::Mat& m) {
    n++;
    m.convertTo(floatVersion, type);
    sum+= floatVersion;
  }
  cv::Mat& TemporalAverage::average() {
    averagedFloat = sum / n;
    averagedFloat.convertTo(averaged, returnType);
    return averaged;
  }

  
}
