#include <conio.h>
#include "general.h"
#include "onitializer.h"
#include "image_grabber.h"
#include "image_type_converter.h"
#include "standard_face_detector.h"
#include "fast_face_detector.h"
#include "image_normalizer.h"
#include "gf_extractor_on_image.h"
#include "multi_boost_live.h"
#include "image_drawer.h"
#include "image_visualizer.h"

using namespace std;

int main()
{
	try
	{
		bool useFastFaceLocator = true;
		bool verbose = true;
		int lineThickness = 2;
		int kindOfFont = CV_FONT_HERSHEY_SIMPLEX;
		double hScale = 1.0;
		double vScale = 1.0;
		int lineWidth = 1;

		int nrm_width = 48;
		int nrm_height = 48;

		int exp[7] =
		{ 1,    //neu     //set to 0 all expressions that will not participate in the
		        1,		//joy			//testing phase and to 1 all that will participate.
		        1,		//sur
		        1,		//sad
		        1,		//fear
		        1,		//ang
		        1 };		//dis

		// MultiBoost Parameters												
		string modelFilename = "model48_all7_500feats_set0_MSL_HistEq_cvNorm_Mag_5Lambdas3to12b_8Orient_newCrop.xml";
		string outputFilename("");

		vector1Ds expressions;
		expressions.push_back("neu");
		expressions.push_back("joy");
		expressions.push_back("sur");
		expressions.push_back("sad");
		expressions.push_back("fear");
		expressions.push_back("ang");
		expressions.push_back("dis");

		vector1Di labels;
		labels.push_back(0);
		labels.push_back(+1);
		labels.push_back(+2);
		labels.push_back(+3);
		labels.push_back(+4);
		labels.push_back(+5);
		labels.push_back(+6);

		vector1Ds active_exp;
		vector1Di active_labels;

		Initializer initializer = Initializer();
		initializer.logInExpressions(active_exp, active_labels, expressions, labels, exp);

		int nExpressions = (int) active_exp.size();

		if (nExpressions < 1)
			throw "<Error> Number of expressions is less than 1.";

		Image imageL1;
		IplImage *cvImage = NULL, *greyImage = NULL, *normalizedImage = NULL;

		bool bPersonInFront = false;
		FaceRegion faceRegion;
		EyePositions eyePositions;
		CvPoint lEyePos, rEyePos;
		vector1Df gFV;
		int label = 0;
		CvFont font;
		CvPoint textPosition;

		cvInitFont(&font, kindOfFont, hScale, vScale, 0, lineWidth);

		//Creation of Objects.
		ImageGrabber grabber(verbose);
		ImageTypeConverter imageConverter(verbose);
		FaceDetector* faceFinder;
		ImageNormalizer normalizer(nrm_width, nrm_height, verbose);
		GFExtractorOnImage gf_extractor;
		MultiBoostLive mbLive(modelFilename, active_labels, verbose);
		ImageDrawer drawer(verbose);
		ImageVisualizer visualizer(verbose);

		if (useFastFaceLocator)
			faceFinder = new FastFaceDetector(verbose);
		else
			faceFinder = new StandardFaceDetector(verbose);

		grabber.startCamera();

		while (1)
		{
			imageL1 = grabber.getImage();

			bPersonInFront = faceFinder->isPersonInFront(imageL1);

			imageConverter.convertFromL1ImageToIplImage(imageL1, &cvImage);

			if (bPersonInFront)
			{
				//Get positions of the eyes.
				eyePositions = faceFinder->getEyePositions();

				lEyePos = eyePositions.lEyePos;
				rEyePos = eyePositions.rEyePos;

				//get information about the boundaries of the face.
				faceRegion = faceFinder->getFaceBoundaries();

				// If necessary, convert cvImage to grey scale.
				if (verbose)
					cout << endl << "converting cvImage to grey scale.";

				greyImage = cvCreateImage(cvGetSize(cvImage), 8, 1);
				if (cvImage->nChannels == 3)
					cvCvtColor(cvImage, greyImage, CV_RGB2GRAY);
				else
					cvCopy(cvImage, greyImage);

				//Normalize image: crop and scale.
				if (verbose)
					cout << endl << "Normalizing image.";

				if (!normalizedImage)
					normalizedImage = cvCreateImage(cvSize(nrm_width, nrm_height), 8, 1);

				normalizer.normalizeImage(greyImage, normalizedImage, lEyePos, rEyePos, faceRegion, false, false);

				//Extract Gabor fetures from normalized image.
				if (verbose)
					cout << endl << "Extracting Gabor features.";

				gf_extractor.extractFeaturesOnImage(normalizedImage, gFV);

				//Classify the image into one of the 7 facial expressions.
				if (verbose)
					cout << endl << "Classifying image.";

				label = mbLive.runLive(gFV);

				if (verbose)
					cout << endl << endl << "EXPRESSION: " << active_exp[label] << endl << endl;

				//Draw cross for left and right eyes
				drawer.drawCross(cvImage, lEyePos, lineThickness, BLUE);
				drawer.drawCross(cvImage, rEyePos, lineThickness, BLUE);

				//Draw face rectangle.
				drawer.drawRectangle(cvImage, faceRegion, lineThickness, BLUE);

				//Draw expression on image.
				textPosition.x = faceRegion.x + faceRegion.width + 20;
				textPosition.y = faceRegion.y + faceRegion.height - 20;
				drawer.drawText(cvImage, active_exp[label].c_str(), textPosition, &font, BLUE);

				gFV.clear();
			}

			visualizer.displayImage(cvImage, "Original");

			cvReleaseImage(&cvImage);
			cvReleaseImage(&greyImage);
		}

		grabber.stopCamera();

		cvReleaseImage(&normalizedImage);

		gFV.~vector();
		expressions.~vector();
		labels.~vector();
		active_exp.~vector();
		active_labels.~vector();
	}
	catch (visg::idt::Exception& exceptionL)
	{
		cout << endl << "what: " << exceptionL.what() << endl << endl;
		cout << endl << "where: " << exceptionL.where() << endl << endl;

		while (!kbhit())
			Sleep(10);
	}
	catch (const char* errorMsg)
	{
		cout << endl << errorMsg << endl;

		while (!kbhit())
			Sleep(10);
	}
}
