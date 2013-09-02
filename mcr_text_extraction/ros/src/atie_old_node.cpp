#include "active_tie/active_camera.h"
#include "active_tie/ov_active_camera.h"
#include "active_tie/sony_dfwvl500_values.h"
#include "active_tie/active_camera_ros.h"
#include "active_tie/acquisition_parameters_optimization.h"
#include "active_tie/ocr_interface.h"
#include "active_tie/tie_ros_interface.h"
#include "atie_old/old_tie_ros_interface.h" 
#include "atie_old/cc_based_tie.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "atie_old_node");
	ros::NodeHandle nh;
	initParameters();
	init();
	atie::ActiveCameraValues* config = new atie::SonyDFWVL500Values();
	atie::ActiveCamera* camera = new atie::ActiveCameraROS("image", NULL, config);
	camera->view(false);
	atie::OVActiveCamera cameraOptim(camera);
	cameraOptim.setOptimizeView(false);
	cameraOptim.setNFramesToAverage(0);
	TIEROSInterface* ri = new OldTIEROSInterface(&cameraOptim);
	ros::spin();
	return 0;
}
