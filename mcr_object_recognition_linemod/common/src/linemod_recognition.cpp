/*
 * linemod_recognition.h
 *
 *  Created on: 7 March 2013
 *      Author: Alexander Hagg
 */

#include "linemod_recognition.h"

LinemodRecognition::LinemodRecognition(std::string& filename, float matching_threshold)
{
	num_classes = 0;
	this->matching_threshold = matching_threshold;
	detector = readLinemod(filename);
	num_modalities = (int) detector->getModalities().size();
}

LinemodRecognition::~LinemodRecognition()
{
}

std::vector<std::string> LinemodRecognition::getObjectList()
{
	return detector->classIds();
}

cv::Ptr<cv::linemod::Detector> LinemodRecognition::readLinemod(const std::string& filename)
{
	cv::Ptr < cv::linemod::Detector > detector = new cv::linemod::Detector;
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	detector->read(fs.root());
	cv::FileNode fn = fs["classes"];

	for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
		detector->readClass(*i);

	ids = detector->classIds();
	num_classes = detector->numClasses();

	printf("Loaded %s with %d classes and %d templates\n", filename.c_str(), num_classes, detector->numTemplates());

	if (!ids.empty())
	{
		printf("Class ids:\n");
		std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	}

	return detector;
}

std::vector<ObjectRegion> LinemodRecognition::recognizeObjects(cv::Mat color_img, cv::Mat depth_img, std::vector<std::string> class_ids)
{

	std::vector<cv::Mat> sources;
	sources.push_back(color_img);
	sources.push_back(depth_img);

	std::vector<cv::linemod::Match> matches;
	std::vector<cv::Mat> quantized_images;
	std::vector<ObjectRegion> objects;

	if (class_ids.size() > 0)
	{
		detector->match(sources, matching_threshold, matches, class_ids, quantized_images);
	}
	else
	{
		return objects;
	}

	int classes_visited = 0;
	std::set<std::string> visited;

	for (int i = 0; (i < (int) matches.size()) && (classes_visited < num_classes); ++i)
	{
		cv::linemod::Match m = matches[i];

		if (visited.insert(m.class_id).second)
		{
			++classes_visited;
			std::vector<cv::linemod::Template> templates = detector->getTemplates(m.class_id, m.template_id);
			objects.push_back(ObjectRegion(m.class_id.c_str(), m.similarity, m.x, m.y, templates, num_modalities));
		}
	}

	return objects;
}

void LinemodRecognition::setMatchingThreshold(float threshold)
{
	this->matching_threshold = threshold;
}

