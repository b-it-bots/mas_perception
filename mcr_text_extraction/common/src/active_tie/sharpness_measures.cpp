/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/sharpness_measures.h"


namespace atie {

  double SharpnessMeasure::getSharpness() {
    return sharpness;
  }

  /* Thresholded Magnitud of Gradient Magnitudes */

  TenengradSM::TenengradSM(int threshold_) {
    setThreshold(threshold_);
  }
  
  TenengradSM::TenengradSM(vector<cv::Mat> gradients_) {
  }

  void TenengradSM::setThreshold(int threshold_) {
    threshold= threshold_;
  }

  double TenengradSM::setGradients(Gradients& gradients_) {
    gradients= gradients_;
    if (!gradients_.isExtended()) {
      throw("Gradients must be set in extended mode!");
    }

    integrals.setImage(gradients.magnitude);
    sharpness= integrals.getSum();
    return sharpness;
  }

  /* Intensity variance sharpness measure */

  double VarianceSM::setImage(cv::Mat& image_) {
    image= image_;
    integrals.setImage(image_);
    return setIntegrals(integrals);
  }

  double VarianceSM::setIntegrals(Integrals& integrals_) {
    integrals_= integrals_;
    sharpness= integrals_.getVar();
    return sharpness;
  }

  double AmplitudeSM::setImage(cv::Mat& image_) {
    integrals.setImage(image_);   
    double mean= integrals.getMean();
    sharpness= 0;
    for (int i= 0; i < image_.cols; i++)
      for (int j=0; j < image_.rows; j++) {
	sharpness+= abs(image_.at<uchar>(j, i) - mean);
      }    
    sharpness /= integrals.getN();
    return sharpness;
  }
};

