/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "atie_old/util.h"


IplImage* multiChannelSobel2D(IplImage* img, IplImage* result, int d, int ssize, int mode) {
  CvSize size= cvSize(img->width, img->height);
  IplImage *b, *g, *r;
  b= cvCreateImage(size, 8, 1);  
  g= cvCreateImage(size, 8, 1);
  r= cvCreateImage(size, 8, 1);
  cvSplit(img, b, g, r, NULL);
  sobel2D(b, b, d, ssize, mode);
  sobel2D(g, g, d, ssize, mode);
  sobel2D(r, r, d, ssize, mode);
  int i,j;
  CvScalar s = cvScalar(0, 0, 0, 0);
  CvScalar bp, gp, rp;
  for (i=0; i < img->height; i++)
    for (j=0; j < img->width; j++) {
      bp= cvGet2D(b, i,j);
      gp= cvGet2D(g, i,j);
      rp= cvGet2D(r, i,j);
      s.val[0]= fmax(bp.val[0], gp.val[0]);
      s.val[0]= fmax(s.val[0], rp.val[0]);	
      //      s.val[0]= (bp.val[0] + gp.val[0] + rp.val[0]) / 3;
      cvSet2D(result, i, j, s);      
    }
  cvReleaseImage(&b); 
  cvReleaseImage(&g); 
  cvReleaseImage(&r);
  return result;
}



IplImage* sobel2D(IplImage* img, IplImage* result,  int d, int s, int mode) {
  IplImage* h = cvCreateImage(cvSize(img->width, img->height),  IPL_DEPTH_16S, 1);
  IplImage* v = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_16S, 1);
  IplImage* tmp = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_16S, 1);
  cvSobel(img, h, 0, d, s);
  cvSobel(img, v, d, 0, s);

  int i, j;
  CvScalar r = cvScalar(0,0,0,0);
  double dx, dy;
  double maxM=-1;
  if (mode == SOBEL_MAGNITUDE) {
    for (i=0; i < img->height; i++)
      for (j=0; j < img->width; j++) {
	dy= cvGet2D(v, i, j).val[0];
	dx= cvGet2D(h, i, j).val[0];
	double magnitude= sqrt(dx*dx + dy*dy);
	r.val[0] = magnitude;
	cvSet2D(result, i, j, r);   
      }
  }
  else {
    for (i=0; i < img->height; i++)
      for (j=0; j < img->width; j++) {
	dy= cvGet2D(v, i, j).val[0];
	dx= cvGet2D(h, i, j).val[0];
	float ang= atan2(dy, dx) * (180.0 / 3.141516);
	if (ang < 0) ang+=360;      
	ang= ang * 255 / 360;
	r.val[0] = ang;
	cvSet2D(result, i, j, r);   
      }
  }
/*   cvConvertScaleAbs(tmp, result, 1, 0); */
  cvReleaseImage(&tmp);
  cvReleaseImage(&h);
  cvReleaseImage(&v);
  return result;
}
