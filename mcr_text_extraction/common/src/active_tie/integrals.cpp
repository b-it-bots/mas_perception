/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/integrals.h"
namespace atie {
  
  Integrals::Integrals() {
    ROI.width= ROI.height= ROI.x= ROI.y= 0;
  }

  void Integrals::setImage(cv::Mat& image_) {
    image= image_;
    cv::integral(image_, sum, sqsum, CV_64F);
    if (!ROI.width || !ROI.height) {
      resetROI();
    } 
  }
  
  void Integrals::resetROI() {
    ROI.x= 0;
    ROI.y= 0;
    ROI.width= image.cols;
    ROI.height= image.rows;
    n= image.cols * image.rows;   
  }

  void Integrals::setROI(cv::Rect& ROI_) {
    ROI.x= ROI_.x;
    ROI.y= ROI_.y;
    ROI.width= ROI_.width;
    ROI.height= ROI_.height;
    n= ROI.width * ROI.height;
  }
  
  double Integrals::getMean() {
    return (getSum() / n);
  }

  double Integrals::getStd() {
    return sqrt(getVar());
  }

  double Integrals::getVar() {
    int sqsum= getSqSum();
    int mean= getMean();
    return (sqsum / n) - (mean * mean);
  }

  double Integrals::quickIntegral(cv::Mat& i_, cv::Rect& r_) {
    int x1= r_.x; 
    int y1= r_.y;
    int x2= x1 + r_.width -1;
    int y2= y1 + r_.height - 1;
    double r= i_.at<double>(y1, x1) +
      + i_.at<double>(y2, x2) - 
      i_.at<double>(y1, x2)  -
      i_.at<double>(y2, x1);
    return r;
  }

  double Integrals::getSum() {
    return quickIntegral(sum, ROI);
  }

  double Integrals::getSqSum() {
    return quickIntegral(sqsum, ROI);
  }
  
  long Integrals::getN() {
    return n;
  }
}
