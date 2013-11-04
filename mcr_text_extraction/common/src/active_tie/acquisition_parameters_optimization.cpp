/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/acquisition_parameters_optimization.h"
/* This dependecy is nasty. Get rid of it in the future */
#include "active_tie/active_camera.h"

namespace atie {
  /* Static members initialization! */
  double FibonacciSearch::goldenRatio= 1.6180339887498948482;        

  CameraParamSC::CameraParamSC(ActiveCamera* camera_, const char* param_) {
    camera= camera_;
    strcpy(param, param_);
    averageNFrames= 0;
    averageCriterionValueP= true;
  }

  int CameraParamSC::getMin() {
    return camera->cameraValues->getMin(param);
  }

  int CameraParamSC::getMax() {
    return camera->cameraValues->getMax(param);
  }

  TenengradSC::TenengradSC(ActiveCamera* camera_, const char* param_) : 
    CameraParamSC(camera_, param_) , gradients(true) { }
  
  double TenengradSC::getValue(int x_) {	
    camera->setParametersV(1, param, x_);
    camera->sync();
    cv::Mat frame= camera->getFrame(); 
    gradients.setImage(frame);
    tenengrad.setGradients(gradients);
    return tenengrad.getSharpness();
  }

  /* Amplitude search criterion */
  AmplitudeSC::AmplitudeSC(ActiveCamera* camera_, char* param_) : 
    CameraParamSC(camera_, param_) {}

  double AmplitudeSC::getValue(int x_) {	
    camera->setParametersV(1, param, x_);
    camera->sync();
    cv::Mat frame= camera->getFrame();
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    amplitude.setImage(gray);	     
    return amplitude.getSharpness();
  }

  VarianceSC::VarianceSC(ActiveCamera* camera_, const char* param_) : 
    CameraParamSC(camera_, param_) {}

  double VarianceSC::getValue(int x_) {	
    camera->setParametersV(1, param, x_);
    camera->sync();
    cv::Mat frame= camera->getFrame();
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    variance.setImage(gray);
    return variance.getSharpness();
  }


  /* Fibonacci search  */

  double FibonacciSearch::fibonacci(int n_) {
    /* Closed form of the Fibonacci recursion */
    return (pow(goldenRatio, n_) - 
	    pow((1.0 - goldenRatio), n_)) / sqrt(5);
  }
  
  FibonacciSearch::FibonacciSearch(SearchCriterion* criterion_) {    
    setCriterion(criterion_);
  }

  void FibonacciSearch::setCriterion(SearchCriterion* criterion_) {
    criterion= criterion_;
  }
	

  int FibonacciSearch::optimize() {  
    if (!criterion) {
      throw("Quality criterion not defined!");
    }
    /* Search values interval */
    int a= criterion->getMin();
    int b= criterion->getMax();
    /* Interval length */
    int L= b - a;
    /* Maximum number of iterations  */
    int N=1;
    while (fibonacci(N) < criterion->getMax()) N++;
    /* int N= 15; */
    int x1, x2;
    double y1, y2;
    int I;
    bool leftSide;
    for (int k=1; k <= N / 2 ; k++) {
      /* New interval length */
      /* Step */
      if (k == 1) {
	I= (b - a) * (fibonacci(N - k - 1) / fibonacci(N - k + 1)); 

	/* Initial displacement at both sides of the search interval
	   and retrieval of two points */
	x1= a + I;
	y1= criterion->getValue(x1);
	x2= b - I;
	y2= criterion->getValue(x2);
      }
      else { /* Next steps retrieve only one poin more */
	if (y1 < y2) { 
	  /* Continue on right side [x1, b] */
	  a= x1;
	  x1= x2;
	  y1= y2;
	  I= (b - a) * (fibonacci(N - k - 1) / fibonacci(N - k + 1)); 
	  x2= b - I;
	  y2= criterion->getValue(x2);

	}
	else {
	  /* Continue on left side [a, x2] */
	  b= x2;
 	  x2= x1;
	  y2= y1;
	  I= (b - a) * (fibonacci(N - k - 1) / fibonacci(N - k + 1)); 
	  x1= a + I;
	  y1= criterion->getValue(x1);
	}
      }
    }
    return ((y1 > y2)? x1 : x2);
  }
}
