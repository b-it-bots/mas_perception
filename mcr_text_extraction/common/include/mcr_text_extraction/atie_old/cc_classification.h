/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __CCCLASSIFICATION__
#define __CCCLASSIFICATION__

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <list>
#include <libsvm/svm.h>
#include <fstream>
#include <map>
#include <vector>
#include <fstream>
#include <iostream>

#include "atie_old/util.h"
#include "atie_old/parameters.h"
#include "atie_old/feature_extractors.h"
#include "atie_old/complex_moments.h"

using namespace std;
/* using namespace Torch; */
using namespace ComplexMoments;

extern bool skipPlane;
#define real float
template<class T>
class FeatureExtractor
{
 protected:
	bool trainingMode;
	int nFeatures;
	svm_node* currentFeatureVector;
	double* normalizationParameters;real* features;

 public:
	FeatureExtractor(char* normalizationParametersPath = NULL, int n = 1);
	void prepare(int nFeatures, double* normalization);
	void printFeatureVector(svm_node* featureVector = NULL);
	int fillSVMInputVector(struct svm_node* featureVector, double* (extractor)(T* ccd, int* n), T* ccd, int fCount = 0);
	double inline normalizeSVMFeature(double value, double featureMin, double featureMax, double lower = -1.0, double upper = 1.0);
	~FeatureExtractor();

};

class CCPool
{
 private:
	CvMemStorage* storage;
	CvSeq* firstCC;
 public:
	CCPool();
	CvSeq* getFirstCC();
	void deleteCC(CvSeq* cc);
	void prepare(IplImage* image);
	~CCPool();
};

class CCBasicFeatureExtractor : public FeatureExtractor<ccRelatedData>
{

	/* private: */
 public:
	ccRelatedData currentCC;
	void setCurrentCC(CvSeq* cc);
	CCBasicFeatureExtractor(char* normalizationParametersPath = NULL, int n = 0, bool forTraining = false);
	~CCBasicFeatureExtractor();
	void init(IplImage* naturalImage);
	void prepare(CCPool *pool_, IplImage* binaryImage);
	void printAllFeatureVectors(CCPool* pool_);
	virtual svm_node* getFeatureVector(CvSeq* cc)= 0;
	void renderCC(ccRelatedData* ccd, IplImage* result, CvScalar* color, bool fillHoles = false, bool clean = true);
};

class CCFeatureExtractorCM : public CCBasicFeatureExtractor
{
 private:
	ComplexRadialPolynomialGenerator* zernike;
	ComplexRadialPolynomialGenerator* pseudoZernike;
	int nZernike, nPseudoZernike;

 public:
	CCFeatureExtractorCM(char* normalizationParametersPath = NULL, int n = 0, int nZernike = -1, int nPseudoZernike = -1, bool forTraining = false);
	svm_node* getFeatureVector(CvSeq* cc);
};

class CCFeatureExtractor : public CCBasicFeatureExtractor
{
 public:
	CCFeatureExtractor(char* normalizationParametersPath = NULL, int n = 0, bool forTraining = false);
	svm_node* getFeatureVector(CvSeq* cc);
};

class Classifier
{
 private:
	int nInstances, nPositives, nFalses;
 public:
	Classifier(char* modelFilePath);
	int classify(svm_node* featureVector, float threshold = -1);
	svm_model* model;
	void resetCounters();
	int getNInstances();
	int getNPositives();
	int getNFalses();
	~Classifier();

};

class CCClassifier : public Classifier
{
 private:
 public:
	/*   This should be private, changed to public to speed up the
	 prototype creation for tomorrow */
	CCBasicFeatureExtractor* featureExtractor;
	CCClassifier(CCBasicFeatureExtractor* fe, char* modelPath);
	CCBasicFeatureExtractor* getFeatureExtractor();
	int classify(CvSeq* cc, float threshold = -1);
	void classifyAll(CCPool* pool_, float threshold = -1);
	int produceImage(CCPool* pool_, IplImage* result);
};

typedef list<BinaryCCRelatedData*> NeighborhoodGraph;

class BCCFeatureExtractor : public FeatureExtractor<BinaryCCRelatedData>
{
 private:
	CCClassifier* classifier;
	NeighborhoodGraph neighborhoodGraph;
	CCPool* pool;
 public:
	BCCFeatureExtractor(char* normalizationParametersPath = NULL, int n = 1);
	void prepare(CCPool* p);
	list<BinaryCCRelatedData*>& getNeighborhoodGraph();
	svm_node* getFeatureVector(BinaryCCRelatedData* b);
	void createNeighborhoodGraph(IplImage* image);
	void filterIndividualCCs();
	void printAllFeatureVectors();
	CCClassifier* getUnaryClassifier();
	void showNeighborhoodGraph(IplImage* image, CvScalar color = cvScalar(200));
};

/* class BCCClassifier : public Classifier { */
/*  private:  */
/*   BCCFeatureExtractor* featureExtractor; */
/*  public: */
/*   BCCClassifier(BCCFeatureExtractor* fe, char* modelFilePath); */
/*   int classify(BinaryCCRelatedData* bd, float threshold= -1); */
/*   void classifyAll(float threshold=-1); */
/*   CvRect* groupCCsIntoRegions(int* n); */
/* }; */

#endif
