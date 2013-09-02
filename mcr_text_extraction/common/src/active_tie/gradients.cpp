/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/gradients.h"

namespace atie {
  
  Gradients::Gradients(bool extendedP_) {
    extendedP= extendedP_;
  }

  void Gradients::setImage(cv::Mat& image_) {    
    image= image_;    
    /* Calculate the gradients in X and Y */
    cv::Sobel(image_, dxU, CV_8U, 1, 0, 1, 1, 128);
    cv::Sobel(image_, dyU, CV_8U, 0, 1, 1, 1, 128);
    /* If the image had multiple channels, we use the maximum gradient
       at each possition  */
    if (image.channels() > 1) {
      mergeChannels(dxU, channelX, maxX);
      mergeChannels(dyU, channelY, maxY);
      dxU= maxX;
      dyU= maxY;
    }    
    if (extendedP) {
      if (orientation.size() != image_.size()) {
	orientation= cv::Mat(image_.rows, image_.cols, CV_32FC1);
	magnitude= cv::Mat(image_.rows, image_.cols, CV_32FC1);
      }
    }
    getSignedGradient();
  }
  
  void Gradients::getSignedGradient() {
    if (dxS.size() != dxU.size()) {
      dxS= cv::Mat(dxU.rows, dxU.cols, CV_32S);
      dyS= cv::Mat(dxU.rows, dxU.cols, CV_32S);
    }

    if(extendedP) {
      for (int i= 0; i < dxS.cols; i++)
	for (int j=0; j < dyS.rows; j++) {
	  int dx =dxU.at<uchar>(j, i) - 128;
	  int dy =dyU.at<uchar>(j, i) - 128;
	  dxS.at<int>(j,i)= dx;
	  dyS.at<int>(j,i)= dy;
	  magnitude.at<float>(j, i)= sqrt(dx*dx + dy*dy);
	  /* Orientation is calculated in degrees */
	  orientation.at<float>(j, i)= atan2(dy, dx) * 180.0f / M_PI;
    	}
    }
    else {
      for (int i= 0; i < dxS.cols; i++)
	for (int j=0; j < dyS.rows; j++) {
	  dxS.at<int>(j,i)= dxU.at<uchar>(j, i) - 128;
	  dyS.at<int>(j,i)= dyU.at<uchar>(j, i) - 128; 
	}      
    }
  }
   
  bool Gradients::isExtended() {
    return extendedP;
  }
  
  void Gradients::setExtended(bool extendedP_) {
    extendedP= extendedP_;
  }


  cv::Mat& Gradients::mergeChannels(cv::Mat& src_, cv::Mat* channels_, cv::Mat& max_) {
    cv::split(src_, channels_);
    max_= cv::max(channels_[0], channels_[1]);
    max_= cv::max(max_, channels_[2]);
    return max_;
  }

  void Gradients::view() {
    cv::namedWindow("image", 0);
    cv::imshow("image", image);

    cv::namedWindow("dxU", 0);
    cv::imshow("dxU", dxU);

    cv::namedWindow("dyU", 0);
    cv::imshow("dyU", dyU);
  }
};

