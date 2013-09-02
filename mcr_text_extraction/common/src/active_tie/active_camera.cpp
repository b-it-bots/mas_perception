/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include "active_tie/active_camera.h"

#include <fstream>
#include <iostream> 
#include <vector>
#include <limits>
#include <sstream>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

namespace atie {
  
  bool ActiveCamera::view(bool viewP_) {
    viewP= viewP_;
    if(viewP) {
      cv::namedWindow("camera", 0); 
      if (frameAvailableP) {
	cv::imshow("camera", currentFrame);
	cv::waitKey(100);
      }
    }
    else {      
      cv::destroyWindow("camera");
    }
    return viewP_;
  }

  void ActiveCamera::sync() {
  }

  cv::Mat& ActiveCamera::getFrame() {
    /* Returns the frame in the camera at the moment of the function
       call. The cv::Mat returned is guaranteed to remain unchanged
       until the next call to getFrame()." */
    frameMutex.lock();
    currentFrame.copyTo(userFrame);    
    frameMutex.unlock();
    return userFrame;
  }
  

  ActiveCameraDir::ActiveCameraDir(char* path_) {
    cameraValues= new ActiveCameraValues();
    viewP= false;
    currentT= 0;
    continuosModeP= false;
    frameAvailableP= false;
    basePath= string(path_);
    indexPath= basePath + "index.txt";
    loadIndex();
  }

  int ActiveCamera::getNParameters() {
    return nParameters;
  }

  void ActiveCameraDir::loadIndex() {
    fstream indexStream(indexPath.c_str(), fstream::in);
    if (!indexStream.is_open()) {
      throw("Could not open index file `" + indexPath + "'");
    }
    string line;
    int nEntries= 0;
    vector<string> strs;
    getline(indexStream, line);
    boost::split(strs, line, boost::is_any_of("\t ,"));
    for(int i=0; i < strs.size(); i++) {
      boost::erase_all(strs[i],"\"");
    }
  
    nParameters= 0;
    for(vector<string>::iterator it= strs.begin() + 1; it != strs.end(); it++) {
      if ((*it) == "t") { 
	columnT= nParameters + 1;
	continue;
	/* Initialize the camera configuration with limit values */
	cameraValues->addParameterDefinition((*it), 
					     numeric_limits<int>::max(),
					     numeric_limits<int>::min());
      }
      parameterColumnIndex[nParameters]= (*it);
      parameterIndex[(*it)]= nParameters++;
    }
    currentParameters= new int[nParameters];
    int p;  
    string filename;
    while (getline(indexStream, line)) {
      nEntries++;
      boost::split(strs, line, boost::is_any_of("\t ,"));
      filename= strs[0];
      boost::erase_all(filename,"\"");
      int i=1;
      int pInd=-1;
      for(vector<string>::iterator it= strs.begin() + 1;
	  it != strs.end(); it++, i++) {
	if (i == columnT) continue;
	pInd++;
	try {
	  p= boost::lexical_cast<int>(*it);
	}
	catch(boost::bad_lexical_cast &) {
	  string error= boost::str(boost::format("Garbage in index file: [%s]")
				   % nEntries);
	  cerr << error << endl;
	  cerr.flush();
	  throw(error);
	}
	string paramName= parameterColumnIndex[pInd];
	cameraValues->observedValue(paramName, p);
	currentParameters[pInd]= p;
      }
      imageIndex[parametersToIndexEntry(currentParameters)]= filename;
    }
    currentImageFileName= filename;
  }

  
  string ActiveCameraDir::parametersToIndexEntry(int* parameters_) {  
    string key;
    for (int i=0; i < nParameters; i++) {
      key+= boost::lexical_cast<string>(parameters_[i]) + '|';
    }
    return key;
  }

  int ActiveCamera::setParametersV(int n_, ...) {
    /* Gathers the parameter names and values from the variadic call
       and calls the overloaded setParameters */
    char* names[n_];
    int values[n_];
    va_list vl;
    va_start(vl, n_);
    for (int i=0; i < n_; i++) {
      char* name= va_arg(vl, char*);
      int value= va_arg(vl, int);
      names[i]= name;
      values[i]= value;
    }
    va_end(vl);    
    return setParameters(n_, names, values);
  }

  int ActiveCameraDir::getParameter(char* name_) {
    return currentParameters[parameterIndex[string(name_)]];
  }

  int ActiveCameraDir::setParameters(int n_, char** names_, int* values_) {
    if (continuosModeP) {
      throw("Parameter changes not allowed in continuos mode!!!");
    }
    int parametersTmp[nParameters];
    memcpy(parametersTmp, currentParameters, nParameters * sizeof(int));
    bool fRequested= false;
    for (int i=0; i < n_; i++) {
      string key(names_[i]);
      if (key == "f") fRequested= true;
      if(!parameterIndex.count(key)) {
	string error= str(boost::format("Requesting unexisting parameter `%s'")
			  % key);
	throw(error);
      }
      /* Set the value in the temp vector */
      int index= parameterIndex[key];
      parametersTmp[index]= values_[i];
    }
    /* If the parameter f is requested we assume this call is being
       performed internally and we do not modify the frame
       number. Otherwise, we start from frame 0 of the requested
       acquisition conditions */
    if (!fRequested) {
      parametersTmp[parameterIndex["f"]]= 0;
    }
    string imageKey= parametersToIndexEntry(parametersTmp);
    if (imageIndex.count(imageKey)) {
      /* A new image needs to be loaded */
      if(imageIndex[imageKey] != currentImageFileName) {
	currentImageFileName= imageIndex[imageKey];
	refreshFrame ();
      }
      /* Update the parameter vector */
      memcpy(currentParameters, parametersTmp, nParameters * sizeof(int));
    }
    else {
      /* The parameter is not defined in the index file! */
      string error= str(boost::format("Invalid parameter value `%s'")
			% imageKey);
      throw(error);
    }
  }

  void ActiveCameraDir::refreshFrame() {
    string path= str(boost::format("%s/%s") % basePath % currentImageFileName);
    currentFrame= cv::imread(path);
    if (currentFrame.empty()) {
      cerr << "Could not load frame `" << path << "'" << endl;
    }
    frameAvailableP= true;
    view(viewP);
  }

  void ActiveCameraDir::setContinuosMode(bool continuosModeP_) {
    if (continuosModeP != continuosModeP_) {
      if (continuosModeP_) 
	currentT= 0;      
    }
    continuosModeP= continuosModeP_;
  }  

  cv::Mat& ActiveCameraDir::getFrame() {
    /* Returns the frame in the camera at the moment of the function
       call. The cv::Mat returned is guaranteed to remain unchanged
       until the next call to getFrame()." */
    
    /* Yield the next frame for this acquisition conditions if one
       exists. Otherwise, revert to the first frame */
    int f= currentParameters[parameterIndex["f"]];
    try {
      setParametersV(1, "f", f+1);
    }
    catch(string& s) {
      setParametersV(1, "f", 0);
    }
    currentFrame.copyTo(userFrame);    
    return userFrame;
  }

}
