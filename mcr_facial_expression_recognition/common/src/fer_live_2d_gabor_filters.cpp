#include "general.h"
#include "initializer.cpp"
#include "face_region_estimator.cpp"
#include "image_normalizer.cpp"
#include "gf_extractor_on_image.cpp"
#include "multi_boost_live.cpp"
#include "image_drawer.cpp"
#include "image_visualizer.cpp"

class FERLive2DGaborFilters
{
 public:

	FERLive2DGaborFilters()
	{
		verbose = true;
		lineThickness = 2;
		kindOfFont = CV_FONT_HERSHEY_SIMPLEX;
		hScale = 1.0;
		vScale = 1.0;
		lineWidth = 1;

		nrm_width = 48;
		nrm_height = 48;

		int exp[7] =
		{ 1,  //neu		// set to 0 all expressions that will not participate in the
		        1,  //joy		// testing phase and to 1 all that will participate.
		        1,  //sur
		        1,  //sad
		        1,  //fear
		        1,  //ang
		        1 };  //dis

		// MultiBoost Parameters
		modelFilename = "../models/model48_all7_500feats_set0_MSL_HistEq_cvNorm_Mag_5Lambdas3to12b_8Orient_newCrop.xml";

		expressions.push_back("neutral");
		expressions.push_back("joy");
		expressions.push_back("surprise");
		expressions.push_back("sadness");
		expressions.push_back("fear");
		expressions.push_back("anger");
		expressions.push_back("disgust");

		labels.push_back(0);
		labels.push_back(+1);
		labels.push_back(+2);
		labels.push_back(+3);
		labels.push_back(+4);
		labels.push_back(+5);
		labels.push_back(+6);

		initializer = new Initializer();
		initializer->logInExpressions(active_exp, active_labels, expressions, labels, exp);

		nExpressions = (int) active_exp.size();

		if (nExpressions < 1)
			throw "<Error> Number of expressions is less than 1.";

		greyImage = NULL;
		normalizedImage = NULL;
		gFV.clear();

		bPersonInFront = false;
		label = 0;
		imgFilename = "";
		color = BLUE;
		frameNr = 0;

		cvInitFont(&font, kindOfFont, hScale, vScale, 0, lineWidth);

		//Creation of objects.
		faceRegionEst = new FaceRegionEstimator(verbose);
		normalizer = new ImageNormalizer(nrm_width, nrm_height, verbose);
		gf_extractor = new GFExtractorOnImage();

		mbLive = new MultiBoostLive(modelFilename, active_labels, verbose);
		drawer = new ImageDrawer(verbose);
		visualizer = new ImageVisualizer(verbose);
	}

	~FERLive2DGaborFilters()
	{
		delete initializer;
		delete faceRegionEst;
		delete normalizer;
		delete gf_extractor;

		delete mbLive;
		delete drawer;
		delete visualizer;
	}

	string recognizeExpression(IplImage* inputImg, CvPoint lEyePos, CvPoint rEyePos)
	{
		string expressionLabel("");

		//   inputImg = cvLoadImage("./adam.jpg", CV_LOAD_IMAGE_COLOR);

		imgToDraw = cvCloneImage(inputImg);

		//get information about the boundaries of the face.
		faceRegion = faceRegionEst->estimateFaceBoundaries(lEyePos, rEyePos);

		// If necessary, convert cvImage to grey scale.
		if (verbose)
			cout << endl << "converting cvImage to grey scale.";

		greyImage = cvCreateImage(cvGetSize(inputImg), IPL_DEPTH_8U, 1);
		if (inputImg->nChannels == 3)
		{
			cout << endl << "the image has color";
			cvCvtColor(inputImg, greyImage, CV_RGB2GRAY);
		}
		else
		{
			cout << endl << "the image is grey.";
			cvCopy(inputImg, greyImage);
		}

		//Normalize image: crop and scale.
		if (verbose)
			cout << endl << "Normalizing image." << endl;

		if (!normalizedImage)
			normalizedImage = cvCreateImage(cvSize(nrm_width, nrm_height), 8, 1);

		cout << endl << "after Normalizing image." << endl;

		normalizer->normalizeImage(greyImage, normalizedImage, lEyePos, rEyePos, faceRegion, false, false);

		//Extract LBP-TOP features from normalized image.
		if (verbose)
			cout << endl << "Extracting Gabor features." << endl;

		gf_extractor->extractFeaturesOnImage(normalizedImage, gFV);

		//Classify the image into one of the 7 facial expressions.
		if (verbose)
			cout << endl << "Classifying image." << endl;

		label = mbLive->runLive(gFV);
		expressionLabel = active_exp[label];

		if (verbose)
			cout << endl << endl << "EXPRESSION: " << active_exp[label] << endl << endl;

		gFV.clear();

		//Draw cross for left and right eyes
		drawer->drawCross(imgToDraw, lEyePos, lineThickness, color);
		drawer->drawCross(imgToDraw, rEyePos, lineThickness, color);

		//Draw face rectangle.
		drawer->drawRectangle(imgToDraw, faceRegion, lineThickness, color);

		//Draw expression on image.
		textPosition.x = faceRegion.x + faceRegion.width + 20;
		textPosition.y = faceRegion.y + faceRegion.height - 20;
		drawer->drawText(imgToDraw, active_exp[label].c_str(), textPosition, &font, color);

		cvReleaseImage(&imgToDraw);
		cvReleaseImage(&greyImage);
		cvReleaseImage(&normalizedImage);

		return expressionLabel;
	}

 private:
	bool verbose;
	int lineThickness;
	int kindOfFont;
	double hScale;
	double vScale;
	int lineWidth;

	int nrm_width;
	int nrm_height;

	vector1Ds expressions;
	vector1Di labels;

	vector1Ds active_exp;
	vector1Di active_labels;

	// MultiBoost Parameters
	string modelFilename;

	Initializer* initializer;

	int nExpressions;

	IplImage *greyImage, *normalizedImage, *imgToDraw;
	vector1Df gFV;

	bool bPersonInFront;
	FaceRegion faceRegion;
	CvPoint lEyePos, rEyePos;
	int label;
	CvFont font;
	CvPoint textPosition;
	stringstream ss;
	string imgFilename;
	CvScalar color;
	int frameNr;

	//Declaration of objects.
	FaceRegionEstimator* faceRegionEst;
	ImageNormalizer* normalizer;
	GFExtractorOnImage* gf_extractor;

	MultiBoostLive* mbLive;
	ImageDrawer* drawer;
	ImageVisualizer* visualizer;
};
