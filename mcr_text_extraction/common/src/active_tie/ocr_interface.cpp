/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include "active_tie/ocr_interface.h"
#include <iostream>
#include <fstream>

using namespace std;

namespace atie {
  void OCRInterface::cleanResults() {
     /* Remove non-alpha-nummeric characters, then replace replace the
	new line characters with `#' to put everything into a single
	line.  Finally, replace any sequence of consecutive `#'s with
	a single blank space and store the results into a file called
	clean.txt */

    system("sed -e 's/[^a-zA-Z0-9 ]//g' < ocr-output.txt | awk '{printf \"%s#\",$0} END {print \"\"}' | sed 's/#\\+/ /g' > ocr-clean.txt");
  }

  string OCRInterface::readResults() {
    fstream results;
    /* Read the results stored in clean.txt */
    results.open("ocr-clean.txt", ios::in);
    string line;
    getline(results, line);
    results.close();
    return line;
  }

  string TesseractOCR::recognize(cv::Mat& image_) {
    /* These are bunch of nasty hacks! */
    cv::imwrite("ocr-input.bmp", image_);
    system ("tesseract ocr-input.bmp ocr-output");  
    cleanResults();
    return readResults();
  }

  string AbbyOCR::recognize(cv::Mat& image_) {
    cv::imwrite("ocr-input.jpg", image_);
    system("abbyyocr -if ocr-input.jpg -of ocr-output.txt");
    cleanResults();
    return readResults();

  }
}
