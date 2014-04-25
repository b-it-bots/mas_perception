#include "mcr_tabletop_segmentation/object_segmentation.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tabletop_segmentation");

	ObjectSegmentation* segmentation = new ObjectSegmentation();

	ros::spin();

	delete segmentation;

	return 0;
}
