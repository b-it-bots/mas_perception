/*
 * datatypes.h
 *
 *  Created on: May 22, 2013
 *      Author: Frederik Hegger
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_


struct Segment3D
{
	Segment3D() : number_of_segments(0), probability(0.0) {};
	pcl::PointCloud<pcl::PointNormal> pcl_cloud;
	unsigned int number_of_segments;
	double probability;
};

struct Segment3DProperties
{
	Segment3DProperties() : probability(0.0) {};
	pcl::PointXYZ centroid;
	double probability;
};

struct Vertex
{
	Vertex() : related_segment_id(0), id(0), probability(0.0) {};
	pcl::PointXYZ point;
	unsigned int related_segment_id;
	unsigned int id;
	double probability;
};

struct Edge
{
	Edge() : eucl_distance(0.0) {};
	double eucl_distance;
};


#endif /* DATATYPES_H_ */
