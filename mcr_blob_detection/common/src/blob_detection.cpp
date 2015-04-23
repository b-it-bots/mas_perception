#include <mcr_blob_detection/blob_detection.h>

BlobDetection::BlobDetection()
{

}

BlobDetection::~BlobDetection()
{

}

int BlobDetection::detectBlobs(IplImage *input_image, IplImage &debug_image, vector<vector<double> > &blobs)
{
    double pose_x;
    double pose_y;
    double pose_theta;
    double blob_area;

    if (!input_image) {
        return -2; // Image not found
    }

    blob_image_ = cvCreateImage(cvGetSize(input_image), IPL_DEPTH_8U, 3);
    gray_image_ = cvCreateImage(cvGetSize(input_image), IPL_DEPTH_8U, 1);

    cvCvtColor(input_image, gray_image_, CV_BGR2GRAY);
    //cvSmooth(gray_image_, gray_image_, CV_GAUSSIAN, 11, 11);
    //cvEqualizeHist(gray_image_, gray_image_);
    cvAdaptiveThreshold(gray_image_, gray_image_, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 15, 0);
    //cvThreshold(gray_image_, gray_image_, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );

    blobs_ = CBlobResult(gray_image_, NULL, 0);
    blobs_.Filter(blobs_, B_EXCLUDE, CBlobGetArea(), B_LESS, min_blob_area_);
    blobs_.Filter(blobs_, B_EXCLUDE, CBlobGetArea(), B_GREATER, max_blob_area_);

    if(debug_mode_){
        cvMerge( gray_image_, gray_image_, gray_image_, NULL, blob_image_ );
    }

    if(blobs_.GetNumBlobs() > 0){

        blobs.resize(blobs_.GetNumBlobs()); // Resize vector to match the number of blobs
        for (int x = 0; x < blobs_.GetNumBlobs(); x++) {
            blobs[x].resize(4); // Resize vector width to hold 4 properties of the blob
            CBlob  temp_blob;
            temp_blob = blobs_.GetBlob(x);
            pose_x = ((temp_blob.MinX() + temp_blob.MaxX()) / 2);
            pose_y = ((temp_blob.MinY() + temp_blob.MaxY()) / 2);
            pose_theta = get_blob_orientation_(temp_blob);
            if (pose_theta > 180) {
                pose_theta = pose_theta - 180;
            }
            blob_area = get_blob_area_(temp_blob);
            blobs[x][0] = pose_x;
            blobs[x][1] = pose_y;
            blobs[x][2] = pose_theta;
            blobs[x][3] = blob_area;
            if (debug_mode_) {
                temp_blob.FillBlob(blob_image_, CV_RGB(0, 255, 0));
                debug_image = *blob_image_;
                cvWaitKey(1);
            }
        }

        cvSetZero(gray_image_);
        cvSetZero(blob_image_);
        cvSetZero(input_image);
        cvReleaseImage(&gray_image_);
        cvReleaseImage(&blob_image_);

        return 1; //Blobs Detected

    } else {
        return -1; //No Blobs Detected
    }

    return 0; //Unidentified Error

}

void BlobDetection::updateDynamicVariables(bool debug_mode, int min_blob_area, int max_blob_area)
{
    debug_mode_ = debug_mode;
    min_blob_area_ = min_blob_area;
    max_blob_area_ = max_blob_area;
    return;
}