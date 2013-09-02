/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "atie_old/feature_extractors.h"

/*==================== Feature extraction functions ==================== */



double* howManyHolesInCC(ccRelatedData* ccd, int* n) {
  /*   Count the number of holes in the given connected component */
  *n=1;
  double* result= new double[1];
  int holes=0;
  /*   The current connected component  */
  CvSeq* ccc;
  CvRect cbr= cvBoundingRect(ccd->cc);
  for(ccc=ccd->cc->v_next; ccc; ccc=ccc->h_next) { 
    CvRect br= cvBoundingRect(ccc);
    holes++;
  }
  result[0]= holes;
  return result;
}

double* getEdgeContent(ccRelatedData* ccd, int* n) {
  *n=1;
  double* result= new double[*n];
  CvRect br= cvBoundingRect(ccd->cc);
  CvScalar binary, gradient;
  double pixelsInComponent=0;
  double edges=0;

  for (int i= br.y; i < br.y + br.height; i++) {
    for (int j= br.x; j < br.x + br.width; j++) {
      GETMC2D(gradient, ccd->gradient, i, j, uchar);
      GETMC2D(binary, ccd->rendered, i, j, uchar);
      /* Perhaps look for a threshold? */
      if (binary.val[0]) {
	pixelsInComponent++;
	if (gradient.val[0] > 10) edges++;
      }
    }
  }
/* Eliminate edges at the contour */
  edges-= ccd->cc->total;
  pixelsInComponent-= ccd->cc->total;
  if (edges) result[0]= 0;
  if (!pixelsInComponent) pixelsInComponent= 0.000001;
  else result[0]= edges / pixelsInComponent;
  return result;
}

double* getEdgeContrast(ccRelatedData* ccd, int* n) {
  double* result= new double[1];  
  (*n)= 1;  
  CvPoint* p;
  CvScalar gradient;
  double edges=0.0000001;
  for (int i=0; i < ccd->cc->total; i++) {
    p= (CvPoint*)cvGetSeqElem(ccd->cc, i);
    GETMC2D(gradient, ccd->gradient, p->y, p->x, uchar);
    if(gradient.val[0] != 0.0f)
      edges++;    
  }
  if (!edges) result[0]= 0;
  else result[0]= edges / ccd->cc->total;
  if (result [0] > 0.95f)  result[0]=1.0f;
  return result;
}

double* howManyCCContains (ccRelatedData* ccd, int* n) {
  /*   Counts how many connected components are contained in this
       contained component */
  double* result= new double[1];  
  (*n)= 1;  
  CvRect tccBR= cvBoundingRect(ccd->cc);  
  for(CvSeq* ccc=ccd->firstCC; ccc; ccc=ccc->h_next) {
    if (ccc == ccd->cc) continue;
    CvRect cccBR= cvBoundingRect(ccc);
    if (cccBR.x >= tccBR.x &&
	cccBR.y >= tccBR.y &&
	cccBR.x + cccBR.width <= tccBR.x + tccBR.width &&
	cccBR.y + cccBR.height <= tccBR.y + tccBR.height) result[0]++;
  }
  return result;
}

double* aspectRatio (ccRelatedData* ccd, int* n) {
  double* result= new double[1];
  *n= 1;
  /*   Returns the aspect ratio of the given connected component */
  CvRect rect= cvBoundingRect(ccd->cc);
  result[0]= fmax((double)rect.width / (double)rect.height,
		  (double)rect.height / (double)rect.width);
  return result;
}

double* getContourRoughnessOpen(ccRelatedData* ccd, int* n) {
  return getContourRoughness(ccd, n, CV_MOP_OPEN);
}

double* getContourRoughnessClose(ccRelatedData* ccd, int* n) {
  return getContourRoughness(ccd, n, CV_MOP_CLOSE);
}

double* getContourRoughness(ccRelatedData* ccd, int* n, int operation) {
  *n= 1;
  double* result= new double[1];
  // Create a copy of the contour with a little extra place for growth
  CvRect roi= cvGetImageROI(ccd->renderedFilled);
  /*   We need a CC rendered without holes */
  IplImage* filled= cvCreateImage(cvSize(roi.width+ 12, roi.height + 12), 8, 1);
  IplImage* filled2= cvCreateImage(cvSize(roi.width+ 12, roi.height + 12), 8, 1);
  cvZero(filled);
  cvZero(filled2);
  /*   Draw the contour centered in the temporary images */
  CvRect filledROI= cvRect(6, 6, roi.width, roi.height);
  cvSetImageROI(filled, filledROI);
  cvSetImageROI(filled2, filledROI);
  cvCopy(ccd->renderedFilled, filled);
  cvCopy(ccd->renderedFilled, filled2);
  cvResetImageROI(filled2);
  cvResetImageROI(filled);
  /*    Apply one the morphological operator to one of the images */
  cvMorphologyEx(filled, filled, NULL, NULL, operation, 1);
  /* Get the difference between both images */
  cvCmp(filled2, filled, filled, CV_CMP_NE);
  /*   Do the counting on the whole filled images */
  double difference= (double)countWhitePixels(filled) / 
    (double) countWhitePixels(filled2);
  cvReleaseImage(&filled);
  cvReleaseImage(&filled2);
  result[0]= (double) difference;
  return result;
}


double* getHomogeneity(ccRelatedData* ccd, int* n) {
  double* result= new double[2];
  *n=1;
  CvRect br= cvBoundingRect(ccd->cc);
  double area= br.width * br.height;
  CvScalar gray, binary;
  double bRMean=0, cCMean=0;
  int cCN=0;
  /* Expand the CC by one pixel in every direction */
  int startX= br.x - 1 >= 0? br.x - 1 : br.x;
  int startY= br.y - 1 >= 0? br.y - 1 : br.y;
  int endX= br.x + br.width + 1 < ccd->rendered->width? br.x + br.width + 1 : 
    br.x + br.width;
  int endY= br.y + br.height + 1 < ccd->rendered->height? br.y + br.height + 1 : 
    br.y + br.height;

  /* Get the mean value of the intensities within the Bounding
     rectangle as well as the CC */
  for (int i= startY; i < endY; i++) {
    for (int j= startX; j < endX; j++) {
      GETMC2D(gray, ccd->gray, i, j, uchar);
      GETMC2D(binary, ccd->rendered, i, j, uchar);
      bRMean+= gray.val[0];
      if (binary.val[0]) {
	cCN++;
	cCMean+= gray.val[0];
      }
    }
  }

  bRMean /= area;
  cCMean /= cCN;

  /* Now get the variance in be Bounding rectangle and in the
     connected component */
  double cCVar= 0, bRVar= 0;
  for (int i= br.y; i < br.y + br.height; i++) {
    for (int j= br.x; j < br.x + br.width; j++) {
      GETMC2D(gray, ccd->gray, i, j, uchar);
      GETMC2D(binary, ccd->rendered, i, j, uchar);
      double bRDifference= gray.val[0] - bRMean;
      bRVar+= (bRDifference * bRDifference) / area;
      if (binary.val[0]) {
	double cCDifference= gray.val[0] - cCMean;
	cCVar+= (cCDifference * cCDifference) / (double) cCN;
      }
    }

  }

  result[0]= sqrt(cCVar);
  return result;
}


double* getNormalizedCentralMoments(ccRelatedData* ccd, int* n) {
  CvMoments m;
  (*n)= 7;
  double* result= new double[*n];
  cvMoments(ccd->rendered, &m, true);
  result[0]= cvGetNormalizedCentralMoment(&m, 2, 0);  
  result[1]= cvGetNormalizedCentralMoment(&m, 1, 1);
  result[2]= cvGetNormalizedCentralMoment(&m, 0, 2);  
  result[3]= cvGetNormalizedCentralMoment(&m, 3, 0);
  result[4]= cvGetNormalizedCentralMoment(&m, 2, 1);  
  result[5]= cvGetNormalizedCentralMoment(&m, 1, 2);
  result[6]= cvGetNormalizedCentralMoment(&m, 0, 3);

  return result;
}


double* getHuMoments(ccRelatedData* ccd, int* n) {
  *n= 7;
/*   Calculates the Hue invariant moments of a connected component */
  double* result= new double[*n];
  CvMoments m;
  CvHuMoments h;
  cvMoments(ccd->rendered, &m, true);
  cvGetHuMoments(&m, &h);

  result[0]= h.hu1; result[1]= h.hu2;
  result[2]= h.hu3; result[3]= h.hu4;
  result[4]= h.hu5; result[5]= h.hu6;
  result[6]= h.hu7;


  /* result[0]= h.hu1;  result[1]= h.hu4; */
  /* result[2]= h.hu5;  result[3]= h.hu6; */
  /* result[4]= h.hu7; */
  return result;
}


double* getOcuppyRatio(ccRelatedData* ccd, int* n) {
  double* result= new double[1];
  *n= 1;
  CvRect br= cvBoundingRect(ccd->cc);  

  result[0]= (double) countWhitePixels(ccd->rendered) / ((float) br.width * (float) br.height);
  return result;
}

int countWhitePixels(IplImage* binaryImage) {
  /*   Counts the number of white pixels in an image. It also considers
     the ROI of the image */

  int whitePixels= 0;
  CvRect roi= cvGetImageROI(binaryImage);
  CvScalar pixel;
  for (int i= roi.y; i < roi.y + roi.height; i++) {
    for (int j= roi.x; j < roi.x + roi.width; j++) {
      GETMC2D(pixel, binaryImage, i, j, uchar);
      if (pixel.val[0] != 0) whitePixels++;
    }
  }
  return whitePixels;
}

double* getCompactness(ccRelatedData* ccd, int* n) {
  *n=1;
  double* result= new double[1];
/*   Ask difference between cc area and bb of cc area */
/* Check the weird values we are getting */  
  CvRect br= cvBoundingRect(ccd->cc);
  float area= br.height * br.width;
  float perimeter= ccd->cc->total;
  result[0]= area / (perimeter * perimeter);
  return result;
}

void inline strokeStatisticsMainHelper (bool& inStrokeRun, int& inStrokeRunWidth, int& nStrokes, 
					list<int>& strokeWidths,
					int& sumStrokeWidths, 
					CvScalar& pixel) {
  if (pixel.val[0]) { // The pixel is part of the CC
    if (inStrokeRun) inStrokeRunWidth++; // We are already within a stroke
    else { // Stroke starts
      inStrokeRun= true;
      inStrokeRunWidth=1;
      nStrokes++;
    }
  }
  else if (inStrokeRun) { // We reached the end of the stroke
    inStrokeRun= false;
    sumStrokeWidths+= inStrokeRunWidth;
    strokeWidths.push_back(inStrokeRunWidth);
  }
}

void inline strokeStatisticsScanLineEndHelper(bool& inStrokeRun, int& inStrokeRunWidth, 
					      int& sumStrokeWidths, list<int>& strokeWidths) {
  /* We reached the image border (Thus of the current stroke if
     any) */
  
  if (inStrokeRun) {
  inStrokeRun= false;
    sumStrokeWidths+= inStrokeRunWidth;
    strokeWidths.push_back(inStrokeRunWidth);
  }
}

void inline strokeStatisticsLastHelper(ccRelatedData* ccd, int& nStrokes, 
				       int& sumStrokeWidths, list<int>& strokeWidths, 
				       double* result) {
    float meanWidth= 0.0f;  
  if (nStrokes)
    meanWidth= sumStrokeWidths / nStrokes;
  /* Remove strokes above the mean and calculate the new mean */
  float newMean= 0;
  list<int>::iterator it;
  for (it= strokeWidths.begin(); it != strokeWidths.end(); it++) {
    if ((*it) > meanWidth) {
      (*it)= -1;
      nStrokes--;
    }
    else {
      newMean+= (*it);
    }
  }
  if (nStrokes) newMean/= nStrokes;
/*   Calculate standard deviation of the stroke */
  
  float stdDev= 0;
  for (it= strokeWidths.begin(); it != strokeWidths.end(); it++) {
    if ((*it) < 0) continue;
    float diff= (*it) - newMean;
    stdDev+= diff * diff;
  }
  if (nStrokes) stdDev /= nStrokes;
  else stdDev=0;
  stdDev= sqrt(stdDev);
  result[0]= newMean;
  result[1]= stdDev;
}


double* getHorizontalStrokeStatistics(ccRelatedData* ccd, int* n) {
  *n= 1;
  double* result= new double[2]; //(float*) calloc(2, sizeof(float));
  CvScalar pixel;
  bool inStrokeRun= false;
  int inStrokeRunWidth= 0;
  int nStrokes=0;
  int sumStrokeWidths=0;
  list<int> strokeWidths;
  CvRect roi= cvGetImageROI(ccd->rendered);
  for (int i= roi.y; i < roi.y + roi.height; i++) {
    for (int j= roi.x; j < roi.x + roi.width; j++) {
      GETMC2D(pixel, ccd->rendered,i, j, uchar);
      strokeStatisticsMainHelper(inStrokeRun,inStrokeRunWidth, nStrokes, 
			     strokeWidths, sumStrokeWidths, pixel);
    } // Inner for loop (Horizontal raster)   
    strokeStatisticsScanLineEndHelper(inStrokeRun, inStrokeRunWidth, sumStrokeWidths, strokeWidths);
  } // Outer for loop (Vertical raster)
  strokeStatisticsLastHelper(ccd, nStrokes, sumStrokeWidths, strokeWidths, result);
  CvRect rect= cvBoundingRect(ccd->cc);
  result[0] /= rect.width;
  result[1] /= rect.width;
  return result;
}

double* getVerticalStrokeStatistics(ccRelatedData* ccd, int* n) {
  *n= 1;
  double* result= new double[2];
  CvScalar pixel;
  bool inStrokeRun= false;
  int inStrokeRunWidth= 0;
  int nStrokes=0;
  int sumStrokeWidths=0;
  list<int> strokeWidths;
  CvRect roi= cvGetImageROI(ccd->rendered);
  for (int j= roi.x; j < roi.x + roi.width; j++) {
    for (int i= roi.y; i < roi.y + roi.height; i++) {
      GETMC2D(pixel, ccd->rendered,i, j, uchar);
      strokeStatisticsMainHelper(inStrokeRun,inStrokeRunWidth, nStrokes, 
			     strokeWidths, sumStrokeWidths, pixel);
    } // Inner for loop    
    strokeStatisticsScanLineEndHelper(inStrokeRun, inStrokeRunWidth, sumStrokeWidths, strokeWidths);
  } // Outer for loop
  strokeStatisticsLastHelper(ccd, nStrokes, sumStrokeWidths, strokeWidths, result);
  CvRect rect= cvBoundingRect(ccd->cc);
  result[0] /= rect.height;
  result[1] /= rect.height;
  return result;
}

/* ==================== Binary features ==================== */

double* getNormalizedCCCentroidDistance(BinaryCCRelatedData* b, int* n) {  
  double* result= getCCCentroidDistance(b, n);
  result[0] /= max(max(b->br1.width, b->br2.width), 
		   max(b->br1.height, b->br2.height));
}

double* getCCCentroidDistance(BinaryCCRelatedData* b, int* n) {
  *n= 1;
  CvRect fccb= cvBoundingRect(b->cc1);
  CvRect sccb= cvBoundingRect(b->cc2);

  double* result= new double[1];
  CvPoint fcccoord, scccoord;
  getCCCenterCoordinates(&fccb, &fcccoord);
  getCCCenterCoordinates(&sccb, &scccoord);
  
  result[0]= sqrt(powf(fcccoord.x - scccoord.x, 2) +
			powf(fcccoord.y - scccoord.y, 2));
  return result;
}

double* getCCCentroidAngle(BinaryCCRelatedData* b, int* n) {
  *n= 1;
  CvRect fccb= cvBoundingRect(b->cc1);
  CvRect sccb= cvBoundingRect(b->cc2);

  double* result= new double[1];
  CvPoint fcccoord, scccoord;
  getCCCenterCoordinates(&fccb, &fcccoord);
  getCCCenterCoordinates(&sccb, &scccoord);
  float angle= atan2(fcccoord.y - scccoord.y, fcccoord.x - scccoord.x) * 180.0 / M_PI;
  if (angle < 0.0f) angle+= 360.0f;  
  result[0]= angle;
  return result;
}

double* getScaleDifference(BinaryCCRelatedData* b, int* n) {
  *n= 1;
  double* result= new double[2];

  double scale1= (float)(b->br1.width * b->br1.height) / 
    (float)(b->image->width * b->image->height);
  double scale2= (float)(b->br2.width * b->br2.height) / 
    (float)(b->image->width * b->image->height);
  result[0]= fabs(scale1 - scale2);
  return result;  
}
double* getOverlapDegree(BinaryCCRelatedData* b, int* n) {
  *n= 2;
  double* result= new double[2];
  result[0]=0;
  result[1]=0;
  /*   First the horizontal overlap */

  if(b->br1.x + b->br1.width >=b->br2.x && b->br1.x + b->br1.width <= b->br2.x + b->br2.width) {
    result[0]= fabs(b->br1.x + b->br1.width - b->br2.x) / min(b->br1.width, b->br2.width);
  }
  if(b->br2.x + b->br2.width >=b->br1.x && b->br2.x + b->br2.width <= b->br1.x + b->br1.width) {
    result[0]+= fabs(b->br2.x + b->br2.width - b->br1.x) / min(b->br1.width, b->br2.width);
  }

  /*   and finally the vertical overlap */
  if(b->br1.y + b->br1.height >=b->br2.y && b->br1.y + b->br1.height <= b->br2.y + b->br2.height) {
    result[1]= fabs(b->br1.y + b->br1.height - b->br2.y) /min(b->br1.height, b->br2.height);
  }

  if(b->br2.y + b->br2.height >=b->br1.y && b->br2.y + b->br2.height <= b->br1.y + b->br1.height) {
    result[1]+= fabs(b->br2.y + b->br2.height - b->br1.y) / min(b->br1.height, b->br2.height);
  }

  if (b->br1.x == b->br2.x && b->br1.x + b->br1.width == b->br2.x + b->br2.width) result[0]= 1;
  if (b->br1.y == b->br2.y && b->br1.y + b->br1.height == b->br2.y + b->br2.height) result[1]= 1;
  result[0]= result[0] > 1? 1 : result[0];
  result[1]= result[1] > 1? 1 : result[1];

  return result;

}


double* getShapeDifference(BinaryCCRelatedData* b, int* n) {  
/*   Estimate the shape difference. Do it by use the Hu moments */
  *n= 1;
  double* result= new double[*n];
  CvMoments m;
  CvHuMoments h1, h2;
  cvSetImageROI(b->image, b->br1);  
  cvMoments(b->image, &m, true);
  double x1= cvGetNormalizedCentralMoment(&m, 2, 0) + 
    cvGetNormalizedCentralMoment(&m, 0, 2);
  double y1= sqrt(pow(cvGetNormalizedCentralMoment(&m, 2, 0) - 
		      cvGetNormalizedCentralMoment(&m, 0, 2),2) +
		  4 * pow(cvGetNormalizedCentralMoment(&m, 1, 1),2));

  cvResetImageROI(b->image);
  cvSetImageROI(b->image, b->br2);  
  cvMoments(b->image, &m, true);
  double x2= cvGetNormalizedCentralMoment(&m, 2, 0) + 
    cvGetNormalizedCentralMoment(&m, 0, 2);
  double y2= sqrt(pow(cvGetNormalizedCentralMoment(&m, 2, 0) - 
		      cvGetNormalizedCentralMoment(&m, 0, 2),2) +
		  4 * pow(cvGetNormalizedCentralMoment(&m, 1, 1),2));  
  cvResetImageROI(b->image);
  result[0]= sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  return result;
}
