/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/active_camera_values.h"


namespace atie {
  void ActiveCameraValues::addParameterDefinition(string name_, int min_,
						  int max_, long FRDelay_) {
    minValues[name_]= min_;
    maxValues[name_]= max_;
    parameterFRDelays[name_]= FRDelay_;
    
  }

  void ActiveCameraValues::observedValue(string name_, int value_) {
    if (value_ < minValues[name_]) minValues[name_]= value_;
    else if (value_ > maxValues[name_]) maxValues[name_]= value_;
  }

  long ActiveCameraValues::getParameterFRDelay(string name_) {
    return parameterFRDelays[name_];
  }
  
  int ActiveCameraValues::translateToSensorValue(string name_, int value_) {
    // Range
    int m= minValues[name_];
    int M= maxValues[name_];
  value_+= m;
  if (value_ < m) value_= m;
  if (value_ > M) value_= M;
  return value_;
  }


  int ActiveCameraValues::getRangeSize(string name_) {
    return (maxValues[name_] - minValues[name_]);
  }
  
  int ActiveCameraValues::getMin(string name_) {
    return 0;
  }

  int ActiveCameraValues::getMax(string name_) {
    return getRangeSize(name_);
  }


};
