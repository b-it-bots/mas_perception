/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


/* Implementation of the Pseudo-Zernike-Moments (PZM) as features from
   binary images, this implementation is targeted for it's use with
   OpenCV */

/* #define NDEBUG  */
#include "atie_old/complex_moments.h"

namespace ComplexMoments {

  vector<double> Factorial::factorials;


  /* ================================================== 
     ImageIterator implementation
     ================================================== */

  ImageIterator::ImageIterator(BaseImage& i) {
    image= &i;
    rewind();       
  }

  bool ImageIterator::atBegin() {
    return (!isAtEnd && x == beginX && y == beginY);
  }

  bool ImageIterator::atEnd() {
    return isAtEnd;
  }

  void ImageIterator::increment() {
    if (isAtEnd) return;    
    do {
      if (x + 1 < image->getWidth()) x++;    
      else if (y + 1 < image->getHeight()) {
	x=0; y++;
      }
      else { 	
	isAtEnd= true;
	return;
      }

    updatePixelData();
    } while(!image->isValidPixel(pixel));

  }
  
  void ImageIterator::rewind() {
    isAtEnd= false;
    x= 0;
    y= 0;
    updatePixelData();
    if (!image->isValidPixel(pixel)) increment();
    /* else updatePixelData(); */
    beginX= x;
    beginY= y;
  }

  void ImageIterator::updatePixelData() {
    pixel.x= x;
    pixel.y= y;
    pixel.realX= x;
    pixel.realY= y;
    image->offsetCoordinates(pixel);
    pixel.value= image->pixelAt(y, x);

    /* From the polar iterator */
    pixel.r= sqrt(pixel.x * pixel.x + pixel.y * pixel.y);
    pixel.theta= atan2(pixel.y - image->getCenterY(), pixel.x - image->getCenterX()) + M_PI;
    image->scaleR(pixel);
    image->shiftTheta(pixel);    
  }

  /* ================================================== 
     BaseImage implementation 
     ================================================== */
  ImageIterator* BaseImage::getIterator() {
    return new ImageIterator(*this);
  }
  

  /* ================================================== 
     Image class implementation
     ================================================== */
  int Image::getHeight() {
    return height;
  }
  int Image::getWidth() {
    return width;
  }
  
  int Image::setHeight(int h) {
    height= h;
    return h;
  }
  int Image::setWidth(int w) {
    width= w;
    return w;
  }

  bool Image::getIsBinary() {
    return isBinary;
  }

  bool Image::setIsBinary(bool b) {
    isBinary= b;
    return b;
  }

  bool Image::isValidPixel(PixelData& p) {    
    return (p.y >=0 && p.y < height && 
	    p.x >= 0 && p.x < width);
  }

  void Image::offsetCoordinates(PixelData& p) {
    /* By default, the image coordinates are not translated */
  }
  
  void Image::scaleR(PixelData& p) {
    /* By default r is not scaled */
  }
  
  void Image::shiftTheta(PixelData& p) {
    /* By default theta is not shifted  */

  }

  void Image::initializeCenter() {
    centerY= height / 2;
    centerX= width / 2;
  }

  int Image::getCenterX() {
    return centerX;
  }

  int Image::getCenterY() {
    return centerY;
  }

  int Image::setCenterX(int x) {
    centerX= x;
    return x;
  }
  
  int Image::setCenterY(int y) {
    centerY= y;
    return y;
  }


  /* ==================================================
     WrappedImage implementation
     ================================================== */

  WrappedImage::WrappedImage(BaseImage* img) {
    image= img;
  }
  
  int WrappedImage::setHeight(int w) {
    return image->setHeight(w);
  }

  int WrappedImage::getHeight() {
    return image->getHeight();
  }

  int WrappedImage::setWidth(int w) {
    return image->setWidth(w); 
  }

  int WrappedImage::getWidth() {
    return image->getWidth();
  }

  bool WrappedImage::setIsBinary(bool b) {
    return image->setIsBinary(b);
  }

  bool WrappedImage::getIsBinary() {
    return image->getIsBinary();
  }

  double WrappedImage::pixelAt(int y, int x) {
    return image->pixelAt(y, x);
  }
  
  void WrappedImage::pixelAt(int y, int x, double value) {
    image->pixelAt(y, x, value);
  }


  int WrappedImage::getCenterX() {
    return image->getCenterX();
  }

  int WrappedImage::getCenterY() {
    return image->getCenterY();
  }

  int WrappedImage::setCenterX(int x) {
    return image->setCenterX(x);
  }

  int WrappedImage::setCenterY(int y) {
    return image->setCenterY(y);
  }

  bool WrappedImage::isValidPixel(PixelData& p){
    image->isValidPixel(p);
  }
  inline void WrappedImage::offsetCoordinates(PixelData& p){
    image->offsetCoordinates(p);
  }
  
  inline void WrappedImage::scaleR(PixelData& p){
    image->scaleR(p);
  }

  inline void WrappedImage::shiftTheta(PixelData& p){
    image->shiftTheta(p);
  }

  void WrappedImage::initializeCenter() {
    image->initializeCenter();
  }


  double GeometricalMoments::getMoment(BaseImage* image, int yOrder, int xOrder) {
    double result=0;
    for (int y= 0; y < image->getHeight(); y++) {
      for (int x= 0; x < image->getWidth(); x++) {
	result+= std::pow((double)x, xOrder) * std::pow((double)y, yOrder) * 
	  image->pixelAt(y, x);
      }
    }
    return result;
  }
  
  /* ==================================================
     NormalizedImage implementation
     ================================================== */
  
  NormalizedImage::NormalizedImage (BaseImage* img, bool escaleToUnitCircle) 
    : WrappedImage(img) {
    maxDistance= 0;
    m00= m01= m10= 0;
    this->escaleToUnitCircle= escaleToUnitCircle;
    /* Note that the moments subscripts are also in terms of yx and
       not xy. It is so to perserve consistecy accross the entire
       library */    
    for (int y= 0; y < image->getHeight(); y++) 
      for (int x= 0; x < image->getWidth(); x++) {
	double intensity= image->pixelAt(y, x);
	m00+= intensity;
	m10+= intensity * y;
	m01+= intensity * x;
      }
    /* Estimate the center of mass */
    initializeCenter();

    /* Find the squared distance to the farthest non-zero pixel
       respect the COM */
    for (int y= 0; y < image->getHeight(); y++) 
      for (int x= 0; x < image->getWidth(); x++) {
	double intensity= image->pixelAt(y, x);
	if (!intensity) continue;
	double squaredDistance= (x - centerX) * (x - centerX) + 
	  (y - centerY) * (y - centerY);
	if (squaredDistance > maxDistance) maxDistance= squaredDistance;
      }
    maxDistance= sqrt(maxDistance);
  } // End of NormalizedImage constructor


  void NormalizedImage::initializeCenter() {
    setCenterY(m10 / m00);
    setCenterX(m01 / m00);
  }

  double NormalizedImage:: getMaxDistance() {
    return maxDistance;
  }

  double NormalizedImage::pixelAt(int y, int x) {
    return image->pixelAt(y, x);;
  }

  void NormalizedImage::pixelAt(int y, int x, double value) {
    image->pixelAt(y, x, value);
  }

  
  void NormalizedImage::offsetCoordinates(PixelData& p) {
    p.y-= centerY;
    p.x-= centerX;
  }

  void NormalizedImage::scaleR(PixelData& p) {
    /* Estimate the distance to the center of mass and divide by the
       distance of the farthest point (if requested) to achieve partial scale
       invariancy */
    if (escaleToUnitCircle) {
      p.r /= maxDistance;
    }
  }
  
  void NormalizedImage::shiftTheta(PixelData& p) {
    /* Meassure the angle respect the center of mass (remember that
       translation has not ocurret yet, so it has to be done */
  }

  bool NormalizedImage::isValidPixel(PixelData& p) {
    /* the normalized distance has to be less than one! */
    if (!escaleToUnitCircle) return image->isValidPixel(p);
    return (p.r  <= 1.0f);
  }

  int NormalizedImage::getCenterX() {return centerX;}
  int NormalizedImage::getCenterY() {return centerY;}
  int NormalizedImage::setCenterX(int x){centerX= x;}
  int NormalizedImage::setCenterY(int y) {centerY= y;}
  void NormalizedImage::setMaxDistance(double distance) {maxDistance= distance;}

  double NormalizedImage::getMass() { 
    return m00; 
  }

  double NormalizedImage::getM10() { 
    return m10; 
  }

  double NormalizedImage::getM01() { 
    return m01; 
  }

  /* ==================================================
     OpenCV wrapper implementation
     ================================================== */
  
  OpenCVImage::OpenCVImage(IplImage* i, bool binary) {
    image= i;
    roc= cvGetImageROI(i);
    width= roc.width;
    height= roc.height;
    isBinary= binary;
    initializeCenter();
  }

  double OpenCVImage::pixelAt(int y, int x) {
    /* Take the data from the first channel only and the ROI! */
    y+= roc.y;
    x+= roc.x;
    double s= ((uchar *)(image->imageData + (y)*image->widthStep))
      [(x)*image->nChannels];
    /* Take into account if the image is binary or not */
    if (!isBinary) return s;
    else return (s > 0? 1.0 : 0.0f);
  }

  void OpenCVImage::pixelAt(int y, int x, double value) {
    /* Translate accordint to the ROI */
    y+= roc.y;
    x+= roc.x;
    ((uchar*)(image->imageData + (y)*image->widthStep))
      [(x)*image->nChannels]= value;
  }

  /* ==================================================
     Factorial lookup table implementation 
     ================================================== */

  double Factorial::fillUpTable(int n) {

    /* Should we allocate more space? A single pre-reallocation
       prevent reallocations to happen during the calculations */
    factorials.reserve(n + 1);
    /* Initialize for factorial(0) */
    if (factorials.size() == 0) factorials.push_back(1);

    /* return 0; */
    /* Calculate factorials up to n */
    double currentFactorial=factorials[factorials.size() -1 ];
    for (int i= factorials.size(); i <= n; i++) {
      currentFactorial= i * factorials[i-1];
      /* Exit if numerical overflow occurs! */
      if (currentFactorial == numeric_limits<double>::infinity()) {
	cerr <<  "Factorial of " << n << endl;
	throw "Numeric overflow in factorial evaluation!";
      }
      factorials.push_back(currentFactorial);
    }
    return currentFactorial;
  }

  double Factorial::value (int n) {  
    return (n > (int)factorials.size() -1)? Factorial::fillUpTable(n) : factorials[n];
  } // End of value

  /* ==================================================
     ComplexRadialPolynomialGenerator implementation 
     ================================================== */

  ComplexRadialPolynomialGenerator::ComplexRadialPolynomialGenerator
    (int maxOrder, bool forPatternRecognition) {
    mode= CRPG_MODE_UPTO;
    this->maxOrder= maxOrder;
    this->forPatternRecognition= forPatternRecognition;
  }


  ComplexRadialPolynomialGenerator::ComplexRadialPolynomialGenerator
    (int* orders, int nOrders, bool forPatternRecognition) {
    mode= CRPG_MODE_GIVEN;    
    this->givenOrders=  orders;
    nFeatures= nOrders;
    features= new ComplexMoment[nFeatures];
    this->forPatternRecognition= forPatternRecognition;
  }

  ComplexRadialPolynomialGenerator::ComplexRadialPolynomialGenerator
    (int n, int m, bool forPatternRecognition) {
    mode= CRPG_MODE_SINGLE;
    int singleN= n;
    int singleM= m;
    nFeatures= 1;
    features= new ComplexMoment[nFeatures];
    features[0].m= singleM;
    features[0].n= singleN;
    this->forPatternRecognition= forPatternRecognition;
  }
  
  void ComplexRadialPolynomialGenerator::initialize() {
    /* Such initialization cannot be done in the constructor since it
       would imply calling some overloaded pure virtual method
       implemented in the subclass; this is not defined in the C++
       standards nor has a consistent behavior among compilers */
    switch (mode) {
    case CRPG_MODE_GIVEN :
      for (int i=0; i < nFeatures; i++) {
	int n= givenOrders[i * 2];
	int m= givenOrders[i * 2 + 1];
	if (!validNM(n, m)) {
	  throw("Improper value for radial polynomial " );
	}
	features[i].n= n;
	features[i].m= m;
      }
      break;
    case CRPG_MODE_UPTO : 
      nFeatures= nMomentsForMaxOrder(maxOrder);
      features= new  ComplexMoment[nFeatures];
      fillFeatureVectorOrdersUpTo();
      break;
    }
    integrals= new complex<double>[nFeatures];
  } // End of initialize

  void ComplexRadialPolynomialGenerator::momentSet(NormalizedImage* image) {
    /* A complex number to store the integration result (initialized to 0) */
    bzero(&integrals[0], nFeatures * sizeof(complex<double>));
    /* We access each pixel only once, it should make things faster */
    int cnt=0;
    for (ImageIterator* it= image->getIterator(); !it->atEnd(); 
	 it->increment()) {
      cnt++;
      float r= it->pixel.r;
      float theta= it->pixel.theta;
      double intensity= it->pixel.value;      
      for (int f= 0; f < nFeatures; f++) {
	int n= features[f].n;
	int m= features[f].m;
	complex<double> complexExp(0.0f, -m * theta);
	integrals[f]+= radialPolynomial(n, m, r) * exp(complexExp) * intensity;
      }  // Interate the moment orders and repetitions
    } // Iterate over the image
    /* Normalize the features (using the common term) and place the
       final values where in the features vector */
    for (int f= 0; f < nFeatures; f++) {      
      int n= features[f].n;
      int m= features[f].m;
      integrals[f] *= commonTerm(n, m);
      integrals[f] /= image->getMass();
      
      features[f].rel = integrals[f].real();
      features[f].img = integrals[f].imag();
      features[f].invariant= 
	sqrt((features[f].rel * features[f].rel) +
	     (features[f].img * features[f].img));
    }
  } // End of momentSet

  int ComplexRadialPolynomialGenerator::getNFeatures() {
    return nFeatures;
  }
  
  ComplexMoment* ComplexRadialPolynomialGenerator::getFeatures() {
    return features;
  }

  void ComplexRadialPolynomialGenerator::reconstruct(NormalizedImage* result, 
						     double mass,
						     double scaling) {
    double reconstructionMass= 0;
    /* Let's go pixel by pixel. This iteration is actually just to get
       the real coordinates in the result image as if fitted to the
       unit circle (or whatever else it might be) */
    for (ImageIterator* it= result->getIterator(); !it->atEnd();
	 it->increment()) {      
      complex<double> sum(0,0);
      /* Integrate over the individual contributions of each moment */
      for (int f=0; f < nFeatures; f++) {
	int n= features[f].n;
	int m= features[f].m;
	complex<double> moment(features[f].rel, features[f].img);
	complex<double> complexExp (0.0f, m * it->pixel.theta);
	/*   /\* Double check this division with the common */
	/*      term. According to my math (which might be wrong) this */
	/*      term is missing in the equations *\/ */
	/*   / commonTerm(n, m); */
	
	complex<double> term= moment * exp(complexExp) 
	  
	  /* Double check this division with the common
	     term. According to my math (which might be wrong) this
	     term is missing in the equations */
	  / (commonTerm(n, m) * radialPolynomial(n, m, it->pixel.r));

	if (isnan(term.real())) continue;
	sum+= term;
      }      
      double intensity= abs(sum.real());
      result->pixelAt(it->pixel.realY , it->pixel.realX, intensity);
      reconstructionMass+= abs(intensity);
    }
    return;
    double newMass=0;
    for (ImageIterator* it= result->getIterator(); !it->atEnd();
	 it->increment()) {      
      complex<double> sum(0,0);
      for (int f=0; f < nFeatures; f++) {
	int n= features[f].n;
	int m= features[f].m;
	complex<double> moment(features[f].rel, features[f].img);
	complex<double> complexExp (0.0f, m * it->pixel.theta);
	complex<double> term= moment * exp(complexExp) *
	  radialPolynomial(n, m, it->pixel.r)
	  / commonTerm(n, m);
	sum+= term;
      }            
      double intensity= abs(sum.real()) /  reconstructionMass;
      newMass+= abs(intensity);
      result->pixelAt(it->pixel.realY , it->pixel.realX, intensity);
    }            
    /* cout << "New mass " << newMass << endl; */
  }

  /* ==================================================
     Zernike Moments implementation 
     ================================================== */  

  Zernike::Zernike(int maxOrder, bool forPatternRecognition) : 
  ComplexRadialPolynomialGenerator(maxOrder, forPatternRecognition) {}

  Zernike::Zernike(int* orders, int nOrders,  bool forPatternRecognition) : 
  ComplexRadialPolynomialGenerator(orders, nOrders, forPatternRecognition) {}

  Zernike::Zernike(int n, int m, bool forPatternRecognition) :
  ComplexRadialPolynomialGenerator(n, m, forPatternRecognition) {}
  
  int Zernike::nMomentsForMaxOrder(int order) {
    if (!forPatternRecognition)
      return (0.5 * (order + 1 ) * (order + 2));
    else {
      int r=0;
      for(int n= 0, i=0; n <= order; n++) {
    	int startM= forPatternRecognition? 0 : -n;
    	for (int m= startM; abs(m) <= n; m++) {
    	  if (validNM(n, m)) r++;
    	}
      }
      return r;
    }
  }

  bool Zernike::validNM(int n, int m) {
    return (n >= 0 && (n - abs(m)) % 2 == 0 && abs(m) <= n);
  }

  double Zernike::radialPolynomial(int n, int m, float r) {
    double sum= 0.0f;    
    int absM= abs(m);
    int nPlusAbsMOver2= (n + absM) / 2;
    int nMinusAbsMOver2= (n - absM) / 2;
    for (int s=0; s <= (n - absM) / 2; s++) {
      /* if s is odd then the power will be negative and positive
	 otherwise */
      int raiseMinus1ToS= s % 2? -1 : 1;
      sum+= raiseMinus1ToS * powl(r, n - s - s) * 
	(Factorial::value(n - s)
	 /
	 (Factorial::value(s) * Factorial::value(nPlusAbsMOver2 - s) *
	  Factorial::value(nMinusAbsMOver2 - s)));
    }
    return sum;
  } // End of radialPolynomial

  double Zernike::commonTerm(int n, int m) {    
    return ((float) n + 1.0f) / M_PI;
  }

  void Zernike::fillFeatureVectorOrdersUpTo() {    
    for(int n= 0, i=0; n <= maxOrder; n++) {
      int startM= forPatternRecognition? 0 : -n;
      for (int m= startM; abs(m) <= n; m++) {	
	if (!validNM(n, m)) continue;
	features[i].n= n;
	features[i++].m =m;
      }
    }
  }
  
  /* ==================================================
     Pseudo-Zernike Moments (PZM) implementation 
    ================================================== */

  PZM::PZM(int maxOrder,  bool forPatternRecognition) : 
  ComplexRadialPolynomialGenerator(maxOrder, forPatternRecognition) {}

  PZM::PZM(int* orders, int nOrders, bool forClassification) : 
  ComplexRadialPolynomialGenerator(orders, nOrders, forPatternRecognition) {}

  PZM::PZM(int n, int m, bool forPatternRecognition) : 
  ComplexRadialPolynomialGenerator(n, m, forPatternRecognition) {}


  bool PZM::validNM(int n, int m) {
    return (n >= 0 && 0 <= abs(m) && abs(m) <= n);
  }

  double PZM::commonTerm(int n, int m) {
    return (2.0f * (n + 1.0f)) / M_PI;
  }

  double PZM::radialPolynomial (int n, int m, float r) {
    double sum= 0.0f;
    int nPlusAbsMPlus1= n + abs(m) + 1;
    int nMinusAbsM= n - abs(m);
    int twoTimesNPlus1= 2 * n + 1;
    for (int s=0; s <= n - abs(m); s++) {
      int raiseMinus1ToS= s % 2? -1 : 1;
      sum+= raiseMinus1ToS * powl(r, n - s) *
	(Factorial::value(twoTimesNPlus1 - s) /
	 (Factorial::value(s) * 
	  Factorial::value(nPlusAbsMPlus1 - s) *
	  Factorial::value(nMinusAbsM - s)));
    }
    return sum;
  } // End of radialPolynomial

  int PZM::nMomentsForMaxOrder(int order) {
    if (!forPatternRecognition)
      return (order + 1) * (order + 1);
    else {
      int r=0;
      for(int n= 0, i=0; n <= order; n++) {
    	int startM= forPatternRecognition? 0 : -n;
    	for (int m= startM; abs(m) <= n; m++) {
    	  if (validNM(n, m)) r++;
    	}
      }
      return r;
    }
  }

  void PZM::fillFeatureVectorOrdersUpTo() {
    int i=0;
    for(int n= 0; n <= maxOrder; n++) {
      int startM= forPatternRecognition? 0 : -n;
      for (int m= startM; abs(m) <= n; m++) {	
	if (!validNM(n, m)) continue;
	features[i].n= n;
	features[i].m =m;
	i++;
      }
    }  
  }
} // End of package scope
