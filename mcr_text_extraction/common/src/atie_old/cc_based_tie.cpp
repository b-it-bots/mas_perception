/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */



#include "atie_old/cc_based_tie.h"

CCBasicFeatureExtractor* fe= NULL;
CCClassifier* classifier= NULL;
BCCFeatureExtractor *be;


float euclideanDistance(CvScalar* p1, CvScalar* p2) {
  return sqrt(pow(p1->val[0] - p2->val[0], 2) +
	      pow(p1->val[1] - p2->val[1], 2) +
	      pow(p1->val[2] - p2->val[2], 2));
}

float getClusterLabel(CvScalar* sample, int k, CvScalar* centroids) {
  int i;  
  float minDistance=9999999;
  float distance;
  int argMin=0;
  for (i=0; i < k; i++) {
    distance= euclideanDistance(sample, &centroids[i]);
    if (distance < minDistance) {
      argMin= i;
      minDistance= distance;
    }
  }
  return argMin;
}


/* ==================== A variaty of segmentators implementations ==================== */
Segmentator::Segmentator(IplImage* plane, IplImage* segmented) {
  currentPlane= plane;
  segmentedImage= segmented;
}


/* ==================== Using Chen-Niblack binarization ==================== */
ChenNiblackSegmentator::ChenNiblackSegmentator(IplImage* img, IplImage* plane, IplImage* segmented, int t, float k)
: Segmentator(plane, segmented) {
  binaryImage= cvCreateImage(cvGetSize(img), 8, 1);
  chenNiblackBinarization(img, binaryImage);
  cvCvtColor(binaryImage, segmentedImage, CV_GRAY2BGR);
  planesDelivered=0;
}

bool ChenNiblackSegmentator::deliverNextPlane() {
  if (planesDelivered == 2) return false;
  if (!planesDelivered) {
    /* Deliver the white plane first */
    cvCmpS(binaryImage, 255, currentPlane, CV_CMP_EQ);
  }
  else {
    /* and then the black plane */
    cvCmpS(binaryImage, 0, currentPlane, CV_CMP_EQ);
  }
  planesDelivered++;
  return true;
}

/* ==================== Detection Results ==================== */


DetectionResults::DetectionResults(int nPlanes_){
  deallocated= false;
  currentPlane= 0;
  currentRectangle= 0;
  nPlanes= nPlanes_;
  rectangles= NULL;
  nPositivesInPlane= (int*) calloc(sizeof(int), nPlanes);
  nInstancesInPlane= (int*) calloc(sizeof(int), nPlanes);
  nFalsesInPlane= (int*) calloc(sizeof(int), nPlanes);
  nRectanglesInPlane= (int*) calloc(sizeof(int), nPlanes);
  planes= (IplImage**) calloc(sizeof(IplImage**),  nPlanes);
  nRectanglesInPlaneSum= (int*) calloc(sizeof(int), nPlanes);    
  nRectangles= 0;
}

DetectionResults::~DetectionResults() {
  if (deallocated) return;
  if (segmentedImage) cvReleaseImage(&segmentedImage);
  deallocated= true;
  free(nRectanglesInPlane);
  free(nRectanglesInPlaneSum);
  free(nPositivesInPlane);
  free(nFalsesInPlane);
  free(nInstancesInPlane);
  free(rectangles);
  for (int i=0; i < nPlanes; i++) {
    cvReleaseImage(&planes[i]);
  }
  free(planes);
}

void DetectionResults::callDestructor() {
  this->~DetectionResults();
}

IplImage* DetectionResults::getPlane(int i) {
  return planes[i];
}

int ChenNiblackSegmentator::getNPlanes() {
  return 2;
}


int DetectionResults::getNPlanes(){
  return nPlanes;
}

int DetectionResults::getNRectanglesInPlane(int p) {
  return nRectanglesInPlane[p];
}
CvRect* DetectionResults::getRectangle(int p, int i) {
  int index= (p == 0?  0 : nRectanglesInPlaneSum[p]);
  return &rectangles[index + i];
}

int DetectionResults::getNRectangles() {
  return nRectangles;
}

void DetectionResults::addResultsNextPlane(CvSeq* cc, IplImage* plane) {
  CvSeq* ccc;
  IplImage* planeCopy= cvCloneImage(plane);
  planes[currentPlane]= planeCopy;
  nRectanglesInPlane[currentPlane]= 0;
  for (ccc= cc; ccc; ccc=ccc->h_next) {
    nRectanglesInPlane[currentPlane]++;
    nRectangles++;
  }
  rectangles= (CvRect*) realloc(rectangles, sizeof(CvRect) * nRectangles);
  
  // Grow the results array new size and populate it 
  for (ccc= cc; ccc; ccc=ccc->h_next) {
    CvRect br= cvBoundingRect(ccc);
    /* CvRect *ccbr= &rectangles[currentRectangle++]; */
    memcpy(&rectangles[currentRectangle++], &br, sizeof(CvRect));
  }
  nRectanglesInPlaneSum[currentPlane]= nRectanglesInPlane[currentPlane];  
  if (currentPlane) 
    nRectanglesInPlaneSum[currentPlane]= nRectanglesInPlane[currentPlane - 1];  
  currentPlane++;
}



void init() {
  if (!be)  be= new BCCFeatureExtractor(NULL, 5);
  printf("Initializing module \n");

  if (fe) delete fe;
  if (classifier) delete classifier;
  if (configurableParameters.zernikeMaxOrder > 0 || configurableParameters.pseudoZernikeMaxOrder > 0) {
    fe= new CCFeatureExtractorCM(configurableParameters.unarySVMNormalization, 
				 configurableParameters.unaryNFeatures, 
				 configurableParameters.zernikeMaxOrder,
				 configurableParameters.pseudoZernikeMaxOrder);
    /* printf("Complex moments Z: %i P: %i\n",  */
    /* 	   configurableParameters.zernikeMaxOrder, */
    /* 	   configurableParameters.pseudoZernikeMaxOrder */
    /* 	   ); */
  }
  else {// Just use Hu moments!
    fe= new CCFeatureExtractor(configurableParameters.unarySVMNormalization, configurableParameters.unaryNFeatures);
    /* printf("Hu moments %i `%s'\n", configurableParameters.unaryNFeatures, */
    /* 	   configurableParameters.unarySVMModel); */
  }
  classifier= new CCClassifier(fe, configurableParameters.unarySVMModel);
}

DetectionResults* processImage(IplImage* imageOrig) {
  int planeID='1';
  IplImage* image= imageOrig;
  CvSize size= cvGetSize(imageOrig);
  /* IplImage* mask= cvCreateImage(size, 8 ,1); */
  IplImage* currentPlane= cvCreateImage(size, IPL_DEPTH_8U, 1);
  IplImage* tmp= cvCreateImage(size, IPL_DEPTH_8U, 1);    
  IplImage* classifierResult= cvCreateImage(size, IPL_DEPTH_8U, 1);
  cvZero(classifierResult);
  /* IplImage* finalResult= cvCreateImage(size, IPL_DEPTH_8U, 3); */
  IplImage* segmentedImage= cvCreateImage(size , IPL_DEPTH_8U, 3);
  Segmentator* segmentator= new ChenNiblackSegmentator(image,currentPlane, segmentedImage);
  DetectionResults* detectionResults= new DetectionResults(segmentator->getNPlanes());
  detectionResults->segmentedImage= segmentedImage;
  /* Look for text like CCs in each color plane */
  int currentNTextAreas=0;
  CCPool pool;
  fe->init(image);
  int plane= 0;
  while(segmentator->deliverNextPlane()) {
    classifier->resetCounters();
    pool.prepare(currentPlane);
    cvResetImageROI(currentPlane);    
    fe->prepare(&pool, currentPlane);
    classifier->classifyAll(&pool, configurableParameters.unaryThreshold);

    /* This need a fix, so that we do not have to reinstantiate the
       BCCFeatureExtractor and classifier */
    cvZero(tmp);
    classifier->produceImage(&pool, tmp);


    if (configurableParameters.doBinaryFiltering) {
      be->prepare(&pool);
      /* BCCClassifier bc(&be, BINARY_SVM_MODEL_FILE); */
      be->createNeighborhoodGraph(tmp);
      /* classifier.produceImage(tmp); */
      /* bc.classifyAll(configurableParameters.binaryThreshold); */
      be->filterIndividualCCs();
    }
    cvZero(tmp);
    classifier->produceImage(&pool, tmp);
    detectionResults->addResultsNextPlane(pool.getFirstCC(), tmp);
    cvResetImageROI(currentPlane);
    cvOr(tmp, classifierResult, classifierResult);
    plane++;
  }
  /* cvSaveImage("/home/g32k1/CybernautRobot/tmp/results2/final.jpg", classifierResult); */
  cvReleaseImage(&currentPlane);
  cvReleaseImage(&tmp);
  cvReleaseImage(&classifierResult);
  return detectionResults;
}
