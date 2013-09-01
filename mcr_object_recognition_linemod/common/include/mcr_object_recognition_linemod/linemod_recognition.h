/*
 * linemod_recognition.cpp
 *
 *  Created on: 7 March 2013
 *      Author: Alexander Hagg
 */

#ifndef LINEMOD_RECOGNITION_H_
#define LINEMOD_RECOGNITION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <string>

class ObjectRegion
{
 public:
	ObjectRegion(std::string class_id, float similarity, int x, int y, std::vector<cv::linemod::Template> templates, int num_modalities)
			: _class_id(class_id),
			  _similarity(similarity),
			  _x(x),
			  _y(y),
			  _templates(templates),
			  _num_modalities(num_modalities)
	{
	}
	std::string _class_id;
	float _similarity;
	int _x, _y;

	std::vector<cv::linemod::Template> _templates;
	int _num_modalities;
};

class LinemodRecognition
{
 public:
	LinemodRecognition(std::string& filename, float matching_threshold);
	~LinemodRecognition();

	cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename);
	std::vector<ObjectRegion> recognizeObjects(cv::Mat color_img, cv::Mat depth_img, std::vector<std::string> class_ids = std::vector<std::string>());
	std::vector<std::string> getObjectList();
	void setMatchingThreshold(float threshold);

 private:
	int num_classes;
	std::vector<std::string> ids;
	float matching_threshold;
	cv::Ptr<cv::linemod::Detector> detector;
	int num_modalities;
};

#endif /* LINEMOD_RECOGNITION_H_ */
