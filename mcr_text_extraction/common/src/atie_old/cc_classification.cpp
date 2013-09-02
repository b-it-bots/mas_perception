/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include "atie_old/cc_classification.h"

/* ==================== CCPool ==================== */
CCPool::CCPool() {
  storage= NULL;
}

CvSeq* CCPool::getFirstCC() {
  return firstCC;
}

void CCPool::deleteCC(CvSeq* cc) {
  if (cc == firstCC) 
    firstCC= cc->h_next;
  if(cc->h_prev) cc->h_prev->h_next = cc->h_next;
  if (cc->h_next) cc->h_next->h_prev = cc->h_prev;
}

void CCPool::prepare(IplImage* image) {
  if (storage) cvReleaseMemStorage(&storage);
  storage= cvCreateMemStorage(0);
  /* Create a copy of the image since CC extraction is a destructive
     operation */
  IplImage* copy= cvCloneImage(image);
  cvFindContours(copy, storage, &firstCC, sizeof(CvContour),
		 CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
  cvReleaseImage(&copy);
}

CCPool::~CCPool() {
  if (storage) cvReleaseMemStorage(&storage);
}



/* ==================== Feature extractor ==================== */
bool skipPlane=false;

CCBasicFeatureExtractor::CCBasicFeatureExtractor(char* normalizationParametersPath, int n, bool forTraining) 
: FeatureExtractor<ccRelatedData>(normalizationParametersPath, n)

{
  trainingMode= forTraining;
  currentCC.plane= NULL;
  currentCC.rendered= NULL;
  currentCC.renderedFilled= NULL;
  currentCC.gray= NULL;
}


CCBasicFeatureExtractor::~CCBasicFeatureExtractor() {
    if (currentCC.gray){
    cvReleaseImage(&currentCC.gradient);
    cvReleaseImage(&currentCC.gray);
    cvReleaseImage(&currentCC.rendered);
    cvReleaseImage(&currentCC.renderedFilled);
  }
}
void CCBasicFeatureExtractor::init(IplImage* naturalImage){
  currentCC.natural= naturalImage;
  
  CvSize size= cvGetSize(naturalImage);  
  if (currentCC.gray){
    cvReleaseImage(&currentCC.gradient);
    cvReleaseImage(&currentCC.gray);
    cvReleaseImage(&currentCC.rendered);
    cvReleaseImage(&currentCC.renderedFilled);

  }
  currentCC.gray= cvCreateImage(size, 8, 1);
  currentCC.gradient= cvCreateImage(size,  8, 1);
  cvCvtColor(naturalImage, currentCC.gray, CV_BGR2GRAY);
  sobel2D(currentCC.gray, currentCC.gradient,  1, 3, SOBEL_MAGNITUDE);  
  currentCC.rendered= cvCreateImage(size, 8, 1);
  currentCC.renderedFilled= cvCreateImage(size, 8, 1);

}

void CCBasicFeatureExtractor::prepare(CCPool *pool_, IplImage* binaryImage) {
  currentCC.plane= binaryImage;
  cvZero(currentCC.rendered);
  cvZero(currentCC.renderedFilled);
  currentCC.firstCC= pool_->getFirstCC();
}


template <class T>
FeatureExtractor<T>::FeatureExtractor(char* normalizationParametersPath, int n){
  normalizationParameters=NULL;
  if (normalizationParametersPath) {
    normalizationParameters= new double[n * 2];
    ifstream f(normalizationParametersPath);
    if(!f) {
      perror("Could not open normalization parameters file");
      printf("%s \n", normalizationParametersPath);
      exit(1);
    }
    string line;
    /* Skip two lines!! */
    getline(f, line); getline(f, line);
    int index;
    float vMin, vMax;
    int i=0;
    while(getline(f, line)) {
      sscanf(line.c_str(), "%d %g %g", &index, &vMin, &vMax);
      normalizationParameters[(index -1) * 2]= vMin;
      normalizationParameters[(index -1) * 2 + 1]= vMax;
      i++;
    }
    f.close();  
  }
    nFeatures= n;
    currentFeatureVector= (struct svm_node*) calloc(nFeatures + 1, 
						    sizeof(struct svm_node));
    currentFeatureVector[nFeatures].index= -1;
    features= (real*) calloc(nFeatures, sizeof(real));
}

template <class T>
FeatureExtractor<T>::~FeatureExtractor() {
  delete normalizationParameters;
  delete features;
}

template <class T>
double inline FeatureExtractor<T>::normalizeSVMFeature(double value, double featureMin, double featureMax, 
				  double lower, double upper) {
/*   Ignore single value feature */
  /* printf("min %f max %f\n", featureMin, featureMax); */
  if (featureMin == featureMax) {
    return value;
  }
  /*   Normalizes the values of a feature within a range of -1 and 1 */  
  if (value == featureMin) return lower;
  else if (value == featureMax)
    return upper;
  else
    return (lower + (upper - lower) * 
	    (value - featureMin)/
	    (featureMax - featureMin));
}



template <class T>
int FeatureExtractor<T>::fillSVMInputVector(struct svm_node* featureVector,  
			double* (extractor) (T*, int* n),
			T* ccd, int fCount) {
  int nVector;
  double* result= extractor(ccd, &nVector);
  for(int i=0; i < nVector; i++) {
    featureVector[i + fCount].index= i + fCount + 1;
    /* If a normalization values vector was given, perform the
       normalization */
    if (normalizationParameters) {
      double min= normalizationParameters[(fCount + i) * 2];
      double max= normalizationParameters[(fCount + i) * 2 + 1];
      double normalized= normalizeSVMFeature(result[i], min, max);
      featureVector[i + fCount].value= normalized;
    }
    else featureVector[i + fCount].value= result[i];
    features[i + fCount]= result[i];
  }
  return nVector;
}


/* ==================== The new feature extractor using complex moments ==================== */


CCFeatureExtractorCM::CCFeatureExtractorCM(char* normalizationParametersPath,
					   int n, int nZernike, int nPseudoZernike,
					   bool forTraining) :
CCBasicFeatureExtractor(normalizationParametersPath, n, forTraining) {
  this->nPseudoZernike= nPseudoZernike;
  this->nZernike= nZernike;
  if (nZernike >= 0) {
    zernike= new Zernike(nZernike, true);
    zernike->initialize();
  } 
  else zernike= NULL;
  if (nPseudoZernike >= 0) {
    pseudoZernike= new PZM(nPseudoZernike, true);
    pseudoZernike->initialize();
  }
  else pseudoZernike= NULL;
}

svm_node* CCFeatureExtractorCM::getFeatureVector(CvSeq* cc) {
  /* fill the other features */
  setCurrentCC(cc);
  CvScalar white= cvScalar(255,255,255,255);
  renderCC(&currentCC, currentCC.rendered, &white, false);
  renderCC(&currentCC, currentCC.renderedFilled, &white, true);

  int index=0;
  /* fill the vector with zernike moments */
  BaseImage* ocv= new OpenCVImage(currentCC.rendered, true);
  NormalizedImage* normalizedImage= new NormalizedImage(ocv, true);
  if (zernike) zernike->momentSet(normalizedImage);
  if (pseudoZernike) pseudoZernike->momentSet(normalizedImage);
  
  if (zernike) {
    ComplexMoment* zFeatures= zernike->getFeatures();
    
    for (int i=0; i < zernike->getNFeatures(); i++) {
      features[index]= normalizeSVMFeature(fabs(zFeatures[i].invariant),
					   normalizationParameters[index* 2],
					   normalizationParameters[index * 2 + 1]);
					   
      currentFeatureVector[index].index= index + 1;
      currentFeatureVector[index].value=features[index];
      index++;
    }
  }
  if (pseudoZernike) {
    ComplexMoment* pFeatures= pseudoZernike->getFeatures();
    for (int i=0; nPseudoZernike >= 0 && 
	   i < pseudoZernike->getNFeatures(); i++) {
      features[index]= normalizeSVMFeature(fabs(pFeatures[i].invariant),
					   normalizationParameters[index* 2],
					   normalizationParameters[index * 2 + 1]);
      currentFeatureVector[index].index= index + 1;
      currentFeatureVector[index].value=features[index];
      index++;
    }
  }
  /* printf("\n"); */

  
  index+= fillSVMInputVector(currentFeatureVector,
  			     getContourRoughnessClose, &currentCC, index);
  index+= fillSVMInputVector(currentFeatureVector,
  			     getContourRoughnessOpen, &currentCC, index);
  index+= fillSVMInputVector(currentFeatureVector,
  			     aspectRatio, &currentCC, index);
  index+= fillSVMInputVector(currentFeatureVector,
  			     howManyCCContains, &currentCC, index);
  index+= fillSVMInputVector(currentFeatureVector,
  			     howManyHolesInCC, &currentCC, index);
  index+= fillSVMInputVector(currentFeatureVector,
  			     getCompactness, &currentCC, index);
  index+= fillSVMInputVector(currentFeatureVector,
  			     getEdgeContrast, &currentCC, index);

  return currentFeatureVector;


  
  
}

/* ==================== The old feature extractor ==================== */

CCFeatureExtractor::CCFeatureExtractor(char* normalizationParametersPath,
				       int n, bool forTraining) :
CCBasicFeatureExtractor(normalizationParametersPath, n, forTraining) {}


void CCBasicFeatureExtractor::printAllFeatureVectors(CCPool* pool_) {
  CvSeq* ccc;
  for(ccc= pool_->getFirstCC(); ccc; ccc= ccc->h_next) {
    svm_node* fv= getFeatureVector(ccc);
    if (trainingMode) {
      CvRect br= cvBoundingRect(ccc);
      if (br.height > 15) printFeatureVector(fv);    
    }
  }
}


template <class T>
void FeatureExtractor<T>::printFeatureVector(svm_node* featureVector) {
  if (!featureVector) featureVector= currentFeatureVector;
  for(int fc=0; fc < nFeatures; fc++) {
    printf("%i:%f ", featureVector[fc].index, featureVector[fc].value);
    }
    printf("\n");
}


void CCBasicFeatureExtractor::renderCC(ccRelatedData* ccd,  IplImage* result, CvScalar* color, bool fillHoles, bool clean) {
/*   Renders the connected components in the rendered IplImage in the
     ccRelatedData structure with a given color. If the CC's holes
     should be filled can be set as a parameter  */

  /*   Store the current ROI */
  if (!ccd) ccd= &currentCC;
  CvRect ROI= cvGetImageROI(result);
  /* Correct this, not necessary as is */
  cvResetImageROI(result);
  if (clean) {
    cvZero(result);
  }
  cvDrawContours(result, ccd->cc, *color, *color, 0, CV_FILLED);
  cvSetImageROI(result, ROI);
  if (not fillHoles) {
    cvSetImageROI(ccd->plane, ROI);
    cvAnd(result, ccd->plane, result);
  }
}

void CCBasicFeatureExtractor::setCurrentCC(CvSeq* cc) {
  currentCC.cc= cc;
  CvRect rect= cvBoundingRect(cc, 0);
  cvResetImageROI(currentCC.rendered);
  cvResetImageROI(currentCC.renderedFilled);
  cvResetImageROI(currentCC.plane);  
  cvSetImageROI(currentCC.rendered, rect);
  cvSetImageROI(currentCC.renderedFilled, rect);
  cvSetImageROI(currentCC.plane, rect);

}

svm_node* CCFeatureExtractor::getFeatureVector(CvSeq* cc){
  setCurrentCC(cc);
  CvScalar white= cvScalar(255,255,255,255);
  renderCC(&currentCC, currentCC.rendered, &white, false);
  renderCC(&currentCC, currentCC.renderedFilled, &white, true);    

  /*  Fill the SVM feature vector */
  int n=0;
  n+= fillSVMInputVector(currentFeatureVector, getHuMoments, &currentCC, 0);
  n+= fillSVMInputVector(currentFeatureVector, getContourRoughnessClose, &currentCC, n);
  n+= fillSVMInputVector(currentFeatureVector, getContourRoughnessOpen, &currentCC, n);
  n+= fillSVMInputVector(currentFeatureVector, aspectRatio, &currentCC, n);
  n+= fillSVMInputVector(currentFeatureVector, howManyCCContains, &currentCC, n);
  n+= fillSVMInputVector(currentFeatureVector, howManyHolesInCC, &currentCC, n);
  n+= fillSVMInputVector(currentFeatureVector, getCompactness, &currentCC, n);
  n+= fillSVMInputVector(currentFeatureVector, getEdgeContrast, &currentCC, n);
  /* Check the distances!!!! */
#ifdef DEBUG_CC_QUERY_USER
  CvRect br= cvBoundingRect(cc);
  if (br.height > 10) {
    char key='\0';
    if (!skipPlane) {
      do{
	key =cvWaitKey(20);
      } while(key != 'y' && key != 'n' && key != 's');
      if (key == 'y')
	printFeatureVector(currentFeatureVector);
      if (key== 's'){
	skipPlane=true;
	printf("Skipping plane\n");
      }
    }
  }
#endif
  return currentFeatureVector;
  }

/* ==================== Connected components classifier ==================== */


CCClassifier::CCClassifier (CCBasicFeatureExtractor* fe, char* modelPath) 
: Classifier(modelPath)
{
  featureExtractor= fe;
}


CCBasicFeatureExtractor* CCClassifier::getFeatureExtractor() {
  return featureExtractor;
}

int CCClassifier::classify(CvSeq* cc, float threshold){  
  /* Ignore too small CCs, this should speed the overall
     classification */  
  CvRect br= cvBoundingRect(cc);
  if (br.height < 10) return -1;
  int classification= Classifier::classify(featureExtractor->getFeatureVector(cc), threshold);
  /* if (classification ==1) */
  /*   huc->calculateDistances(&featureExtractor->currentCC); */
    
  return classification;
}

Classifier::Classifier (char* modelFilePath) {
  resetCounters();
  model= NULL;
  model= svm_load_model(modelFilePath);
  if(!model) {
    cerr << modelFilePath << " : " << endl;
    perror("Could not load the model file");
    printf("%s \n", modelFilePath);
    exit(1);
  }
}

int Classifier::classify(svm_node* featureVector, float threshold) {
  /* Classifies a feature vector using the current model. If threshold
     = 0, the classification will be done using the sign function of
     the result of the svm equatio; However, if threshold > 0, the
     classification will return 1 only if the probability estimate for
     positive class of the feature vector is >= than the threshold and
     0 otherwise. If the threshold < 1, the classification is bypassed
     and all instances get a positive label (1) */
  if(threshold < 0) return 1;
  /* Preconditions fullfilled? */
  if (threshold > 0 && !svm_check_probability_model(model)) {
    fprintf(stderr, "classify: Probability threshold given but the model does not support "
	    "probability estimates!\n");
    return -1;
  }
  nInstances++;
  /* Use probability estimates? */
  int result;
  if (threshold > 0) {
    double probabilities[2] = {0};
    svm_predict_probability(model, featureVector, probabilities);
    result= probabilities[0] >= threshold? 1 : 1;
  }  
  /* Just use the sign function */
  result= svm_predict(model, featureVector);
  if (result > 0) {
    nPositives++;
    return result;
  }
  else {
    nFalses++;
    return result;
  }
}


void Classifier::resetCounters() {
  nInstances= nPositives= nFalses= 0;
}

int Classifier::getNInstances() {
  return nInstances;
}

int Classifier::getNPositives() {
  return nPositives;
}

int Classifier::getNFalses() {
  return nFalses;
}

Classifier::~Classifier() {
  svm_free_model_content(model);
}

void CCClassifier::classifyAll(CCPool* pool_, float threshold) {  
  for (CvSeq* ccc= pool_->getFirstCC(); ccc; ccc= ccc->h_next) {
    if (classify(ccc, threshold) != 1) {
/*       Remove item:  */
/* 	 Three operations have to be performed to preserve the */
/* 	 integrity of the sequence: If the pointer is the to the first */
/* 	 element, it has to be reassigned. Complementary, the links of */
/* 	 the previous and next elements have to be set so that they */
/* 	 point to each other, skipping the element that is being */
/* 	 removed (In the middle)       */
      /* if (ccc == featureExtractor->currentCC.firstCC) */
      /* 	featureExtractor->currentCC.firstCC= ccc->h_next; */
      /* if(ccc->h_prev) ccc->h_prev->h_next = ccc->h_next; */
      /* if (ccc->h_next) ccc->h_next->h_prev = ccc->h_prev; */
      pool_->deleteCC(ccc);
    }
  }
}

int CCClassifier::produceImage(CCPool* pool_, IplImage* result){
  for (CvSeq* ccc= pool_->getFirstCC(); ccc; ccc= ccc->h_next) {
    CvRect br= cvBoundingRect(ccc);
    cvSetImageROI(result, br);
    featureExtractor->setCurrentCC(ccc);
    featureExtractor->renderCC(NULL, result, &cvScalar(255), false, false);
    cvResetImageROI(result);
  }
}

/* ==================== Binary CC classification ==================== */

/* BCCClassifier::BCCClassifier(BCCFeatureExtractor* fe, char* modelFilePath) */
/* : Classifier(modelFilePath) { */
/*   featureExtractor= fe; */
/* } */


/* int BCCClassifier::classify(BinaryCCRelatedData* bd, float threshold) { */
/*   svm_node* fv= featureExtractor->getFeatureVector(bd); */
/*   return Classifier::classify(fv, threshold); */
/* } */
/* void BCCClassifier::classifyAll(float threshold) { */
/*   NeighborhoodGraph& l= featureExtractor->getNeighborhoodGraph(); */
/*   NeighborhoodGraph::iterator it; */
/*   for (it= l.begin(); */
/*        it != l.end(); */
/*        it++) { */
/* /\*     svm_node* fv=  featureExtractor->getFeatureVector(*it); *\/ */
/*     if (classify(*it, threshold) != 1) { */
/*       /\* Remove from the list. We have to account for the iterator */
/* 	 integrity as well as the list integrity *\/ */
/*       it= l.erase(it); */
      
/*     } */
/*   } */
/* } */


/* ==================== Binary CC feature extraction ==================== */

BCCFeatureExtractor::BCCFeatureExtractor(char* normalizationParametersPath, int n)
: FeatureExtractor<BinaryCCRelatedData>(normalizationParametersPath, n)


{
}

CCClassifier* BCCFeatureExtractor::getUnaryClassifier() {
  return classifier;
}

void BCCFeatureExtractor::prepare (CCPool* p){
  pool= p;
  /* classifier= c; */
}

void BCCFeatureExtractor::filterIndividualCCs() {
/*   Delete all CCs not in the neighborhood graph from the CC's list
     of the unary feature extractor */
/*   printf("Individual CC filtering\n");   */
  for(CvSeq* cc= pool->getFirstCC(); cc ; cc= cc->h_next) {
    bool found= false;    
    NeighborhoodGraph::iterator it;
    for(it= neighborhoodGraph.begin();
	it != neighborhoodGraph.end(); it++) {
      BinaryCCRelatedData* bd= (*it);
      if (bd->cc1 == cc || bd->cc2== cc) {
	found= true;
	break;
      }
    }
     if (!found) {
       pool->deleteCC(cc);
       /* if (cc == classifier->featureExtractor->currentCC.firstCC) */
       /* 	 classifier->featureExtractor->currentCC.firstCC= cc->h_next; */
       /* if(cc->h_prev) cc->h_prev->h_next = cc->h_next; */
       /* if (cc->h_next) cc->h_next->h_prev = cc->h_prev; */
    }
  }
}

void BCCFeatureExtractor::createNeighborhoodGraph(IplImage* image) {
  for (CvSeq* fcc= pool->getFirstCC(); fcc; fcc= fcc->h_next) {
    for (CvSeq* scc= fcc->h_next; scc; scc= scc->h_next) {
      int n;
      
      BinaryCCRelatedData* instance= new BinaryCCRelatedData();
      instance->cc1= fcc;
      instance->cc2= scc;
      instance->br1= cvBoundingRect(fcc);
      instance->br2= cvBoundingRect(scc);
      instance->image= image;
      /* Distance between the components centers */

      double distance= getCCCentroidDistance(instance, &n)[0];
      double angle= getCCCentroidAngle(instance, &n)[0];
      double angDisp= abs(angle - 90);
      printf("Centroids angle %g\n", angle);
      /* Linkage rule */
      if ((distance <  1.8 * min(max(instance->br1.width, instance->br1.height),
				 max(instance->br2.width, instance->br2.height))) &&
	  angDisp > 20.0f){
	neighborhoodGraph.push_front(instance);
      }
    }
  }
}

svm_node* BCCFeatureExtractor::getFeatureVector(BinaryCCRelatedData* b) {
  fillSVMInputVector(currentFeatureVector, getScaleDifference, b, 0);
  fillSVMInputVector(currentFeatureVector, getOverlapDegree, b, 1);
  fillSVMInputVector(currentFeatureVector, getShapeDifference, b, 3);
  fillSVMInputVector(currentFeatureVector, getNormalizedCCCentroidDistance, b, 4);
  return currentFeatureVector;
}

NeighborhoodGraph& BCCFeatureExtractor::getNeighborhoodGraph() {
  return neighborhoodGraph;
}

void BCCFeatureExtractor::printAllFeatureVectors(){
    NeighborhoodGraph::iterator it;
    for (it= neighborhoodGraph.begin(); it != neighborhoodGraph.end(); it++) {
      printFeatureVector(getFeatureVector((*it)));
    }
}

void BCCFeatureExtractor::showNeighborhoodGraph(IplImage* image, CvScalar color){
  NeighborhoodGraph::iterator it;
  for (it= neighborhoodGraph.begin(); it != neighborhoodGraph.end(); it++) {
    CvPoint fcccoord, scccoord;
    CvRect fccb= cvBoundingRect((*it)->cc1);
    CvRect sccb= cvBoundingRect((*it)->cc2);
    getCCCenterCoordinates(&fccb, &fcccoord);
    getCCCenterCoordinates(&sccb, &scccoord);
    cvLine(image, fcccoord, scccoord, color, 2);
    cvCircle(image, fcccoord, 2, color);
    cvCircle(image, scccoord, 3, color);
  }
}

void getCCCenterCoordinates(CvRect* bb, CvPoint* p){
  p->x= ((bb->x + bb->x + bb->width) / 2);
  p->y= ((bb->y + bb->y + bb->height) / 2);
}


/* CvRect* BCCClassifier::groupCCsIntoRegions(int* n) { */
/*   /\* Definitely we have to split this function into something more */
/*      manageable. it is too complex (and long) *\/ */

/*   /\* Count the number of remaining connected components and create */
/*      the mappings *\/ */
/*   CvSeq* firstCC= featureExtractor->getCCs(); */
/*   /\* How many CCs still there? *\/ */
/*   int nCC= 0; */
/*   map<CvSeq*, int> pointerToIndex; map<int, CvSeq*> indexToPointer; */
/*   int i=0; */
/*   for(CvSeq* cc= firstCC; cc; cc= cc->h_next, nCC++, i++) { */
/*     pointerToIndex[cc]=i; */
/*     indexToPointer[i]=cc; */
/*   } */
/* /\* Create the nCCxnCC matrix. Mmm perhaps we should think of some */
/*    sparse representation? *\/ */
/*   short int conectivityMatrix[nCC][nCC]; */
/* /\* Initialize the matrix elements to 0 *\/ */
/*   bzero(&conectivityMatrix[0][0], sizeof(short int) * nCC * nCC); */
/*   /\* And the membership pointers vector(initialized to NULL) A */
/*    membership maps the index number of a cc to the cc pointer*\/ */
/*   map<int, CvSeq*>* memberships[nCC]; */
/*   bzero(&memberships[0], sizeof(map<int, CvSeq*>*) * nCC); */
/*   /\* Fill the connectivity matrix with 1's according to the */
/*      unidirectional links *\/ */
/*   NeighborhoodGraph::iterator it; */
/*   NeighborhoodGraph& neighborhoodGraph= featureExtractor->getNeighborhoodGraph(); */
/*   int ones=0; */
/*   for(it= neighborhoodGraph.begin(); it != neighborhoodGraph.end(); it++) { */
/*     BinaryCCRelatedData* bd= (*it); */
/*     int index1= pointerToIndex[bd->cc1]; */
/*     int index2= pointerToIndex[bd->cc2]; */
/*     if (!conectivityMatrix[index1][index2] && */
/* 	!conectivityMatrix[index2][index1]) conectivityMatrix[index1][index2]=1; */
/*   } */
  
/*   /\* A list of all memberships *\/ */
/*   list<map<int, CvSeq*>*> allMemberships; */
/*   for (int i=0; i < nCC; i++) { */
/*     /\* Reuse an existing membership list or instantiate one if */
/*        needed *\/ */
/*     if (!memberships[i])  { */
/*       memberships[i] = new map<int, CvSeq*>; */
/*       (*memberships[i])[i]= indexToPointer[i]; */
/*       allMemberships.push_back(memberships[i]); */
/*     } */
/*     for(int j=0; j < nCC; j++) { */
/*       if (conectivityMatrix[i][j]){ */
/* 	memberships[j]= memberships[i]; */
/* 	(*memberships[i])[j]= indexToPointer[j]; */
/*       } */

/*     } */
/*   } */
/*   *n= allMemberships.size(); */
/*   CvRect* boundingRectangles= new CvRect[*n]; */

/*   /\*  Transform the memberships into a vectors of connected */
/*       components *\/ */
/*   /\* First iterate the list of memberships *\/ */
/*   list<map<int, CvSeq*>*>::iterator mit; */
/*   for (i=0, mit= allMemberships.begin(); */
/*        mit != allMemberships.end(); mit++, i++) { */
/* /\* Iterate the CCs in this membership, because of how the memberships */
/*    are stored, we are interested in the 'second' element of the */
/*    iterator, which contains the pointer to the CC *\/ */
/*     map<int, CvSeq*>::iterator cit; */
/*     CvRect* br= new CvRect; */
/*     /\* We have to double deferentiate the mit, it is a pointer to a */
/*        pointer! *\/ */
/*     for(cit= (**mit).begin(); cit != (**mit).end(); cit++) { */
/*       /\* The first time, just assign the CC bounding rect to br *\/ */
/*       if (cit == (**mit).begin()) { */
/* 	memcpy((void*)br, (void*)&cvBoundingRect((*cit).second), sizeof(CvRect)); */
/*       } */
/*       /\* Otherwise, br is the maximum bounding rect for the current */
/* 	 value of br and the bounding rect of the current CC *\/ */
/*       else { */
/* 	memcpy((void*)br, (void*) &cvMaxRect(&cvBoundingRect((*cit).second), br), sizeof(CvRect)); */
/*       } */
/*     } */
/*     memcpy(&boundingRectangles[i], br, sizeof(CvRect)); */
/*   } */
/* /\* So perhaps the recursive solution would have been better after all */
/*    :P *\/ */
/*   return boundingRectangles; */
/* } */




#ifdef USE_MAIN
int main(int argc, char** argv) {
  char* binaryFilePath= argv[1];
  char* naturalFilePath= argv[2];
  int mode= atoi(argv[3]);
  IplImage* binaryImage= cvLoadImage(binaryFilePath, 0);
  IplImage* naturalImage= cvLoadImage(naturalFilePath, 1);
  CCPool pool;  
  if (mode == 1) {
    /* CCBasicFeatureExtractor* fe= new CCFeatureExtractor(NULL, UNARY_CLASSIFICATION_N_FEATURES, true); */
    /* printf( "Creating F extractor !\n"); */
    CCBasicFeatureExtractor* fe= new CCFeatureExtractorCM(NULL,73,-1,10
, true);
    /* printf( "Done F extractor !\n"); */
    CCClassifier classifier= CCClassifier(fe, UNARY_SVM_MODEL_FILE);
    pool.prepare(binaryImage);
    fe->init(naturalImage);
    fe->prepare(&pool, binaryImage);
    fe->printAllFeatureVectors(&pool);
    
  }
  /* else if (mode == 2) { */
  /*   CCBasicFeatureExtractor fe= CCBasicFeatureExtractor(UNARY_SVM_NORMALIZATION_FILE, */
  /* 					      UNARY_CLASSIFICATION_N_FEATURES); */
  /*   CCClassifier classifier= CCClassifier(&fe, UNARY_SVM_MODEL_FILE); */
  /*   classifier.prepare(binaryImage, naturalImage); */
    
  /*   BCCBasicFeatureExtractor be(NULL, 5); */
  /*   be.prepare(&classifier); */
  /*   classifier.classifyAll(&pool); */
  /*   be.createNeighborhoodGraph(binaryImage); */
  /*   be.printAllFeatureVectors(); */
  /* } */
  return 0;
}
 
#endif
