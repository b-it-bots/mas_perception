/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include "atie_old/cc_based_tie.h"

int main(int argc, char** argv) {  
  initParameters();
  init();

  IplImage* imageOrig= cvLoadImage(argv[1], -1);
  DetectionResults* r= processImage(imageOrig);
  cvNamedWindow("result", 0);
  for(int i=0; i < r->nPlanes; i++) {
    cvNot(r->planes[i], r->planes[i]);
    cvShowImage("result", r->planes[i]);
    while(cvWaitKey() != '\n') {}
  }
  cvReleaseImage(&imageOrig);
  return 0;
}
