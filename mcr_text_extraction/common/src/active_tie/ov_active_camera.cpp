/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/ov_active_camera.h"

namespace atie {
  OVActiveCamera::OVActiveCamera(ActiveCamera* camera_) {    
    camera= camera_;
    nFramesToAverage= 0;
    optimizeViewP= false;
    focusCriterion= new TenengradSC(camera_, "focus");
    irisCriterion= new VarianceSC(camera_, "iris");
    brightnessCriterion= new TenengradSC(camera_, "brightness");
  }
  
  int OVActiveCamera::setParameters(int n_, char** names_, int* values_) {
    return camera->setParameters(n_, names_, values_);
  }

  cv::Mat& OVActiveCamera::getFrame() {
    if(!nFramesToAverage) return camera->getFrame();

    temporalAverage.init(camera->getFrame());

    for (int i=0; i < nFramesToAverage; i++) {
      temporalAverage.add(camera->getFrame());
    }
    return temporalAverage.average();
  }

  bool OVActiveCamera::setOptimizeView(bool optimizeViewP_) {
    optimizeViewP= optimizeViewP_;
    return optimizeViewP;
  }

  void OVActiveCamera::optimizeView() {
    if (!optimizeViewP) return;
    /* Set iris to the middle value and disable the brightness  */
    int irisMin= camera->cameraValues->getMin("iris");
    int irisMax= camera->cameraValues->getMax("iris");
    camera->setParametersV(1, "iris", (int)(irisMin + (5*(irisMax - irisMin) / 8)));
    camera->sync();
    /* Search for the optimal focus */
    fibonacciSearch.setCriterion(focusCriterion);
    int focusOptim= fibonacciSearch.optimize();    
    camera->setParametersV(1, "focus", focusOptim);
    camera->sync();
    /* Search for the optimal iris */
    fibonacciSearch.setCriterion(irisCriterion);
    int irisOptim= fibonacciSearch.optimize();
    camera->setParametersV(1, "iris", irisOptim);
    camera->sync();
  }

  bool OVActiveCamera::view(bool viewP_) {
    camera->view(viewP_);
  }
  void OVActiveCamera::sync() {
    camera->sync();
  }

  int OVActiveCamera::setNFramesToAverage(int n_) {
    nFramesToAverage= n_;
  }

  int OVActiveCamera::getNFramesToAverage() {
    return nFramesToAverage;
  }

  int OVActiveCamera::getParameter(char* name_) {
    return camera->getParameter(name_);
  }
  
}
