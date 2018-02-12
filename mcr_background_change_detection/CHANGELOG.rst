^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcr_background_change_detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* bsmog\_ backgroundSubstractor class changed to Ptr since it is a Abstract class contructor and apply functions are override
* removing CATKIN_IGNORE and vesion requirement in CMakeLists
* Merge branch 'kinetic' of mas.b-it-center.de:mas-group/mas_common_robotics into kinetic
* Add CATKIN_IGNORE files to packages that don't build
* Merge branch 'indigo' of mas.b-it-center.de:mas-group/mas_common_robotics into indigo
* Merge branch 'fix/continuous-integration' into 'indigo'
  Fix/continuous integration
  See merge request !243
* Add install of dynamic reconfigure files
* Merge branch 'erl-to-indigo' into 'indigo'
  Erl to indigo
  See merge request !227
* [background change] update default and max background change threshold
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_common_robotics into indigo
* Merge branch 'rename_image_topic' into 'hydro'
  Rename image topic
  - rename image topic to rgb/image_raw
  See merge request !148
* rename image topics from realsense camera to be similar to those from openni driver
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_common_robotics into hydro
  Conflicts:
  mcr_navigation/mcr_relative_base_controller/ros/src/mcr_relative_base_controller_ros/direct_base_controller_coordinator.py
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_common_robotics into wait_for_moveit
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_common_robotics into indigo
* Merge branch 'rename_tower_to_arm_cam' into 'hydro'
  Rename tower to arm cam
  Renaming "tower_cam3D" to "arm_cam3D"
  See merge request !134
* [Renaming camera] Renamed all tower_cam3D to arm_cam3D
  Commandline grep and xargs was used to replace.
* Merge branch 'hydro' into indigo
  Conflicts:
  mcr_controllers/mcr_trajectory_controller/ros/include/mcr_trajectory_controller/hardware_interface_adapter.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/pos_vel_acc_state.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/quintic_spline_segment.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/trajectory_interface.h
  mcr_controllers/mcr_trajectory_controller/ros/src/trajectory_controller.cpp
* [formatting] replace tabs by spaces, indentation and ending spaces
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_common_robotics into moveit_interpolation_planner_plugin_revised_55
* Merge branch 'hydro' into indigo
* Merge branch 'hydro' into indigo
* [background change detection] export library
* Merge branch 'hydro' into indigo
  Conflicts:
  mcr_navigation/mcr_recovery_behaviors/package.xml
* [lint] update dependencies
  to pass the catkin lint test
* Merge branch 'hydro' into indigo
  Conflicts:
  repository.debs
* Merge branch 'hydro' of git-pi:b-it-bots/mas_common_robotics into hydro
* Merge branch 'hydro' into twist_demux
* Merge branch 'fix_issues' into 'hydro'
  Topic remap updates / Some Issue fixes
  See merge request !76
* [mcr_background_change_detection]Update launch file and fix bug
* Merge branch 'background_change_detection' into 'hydro'
  Background change detection
  Node to detect moving objects in a 2D scene - v1.0
  Can be used to
  1. detect change in the static scene
  2. isolate only the moving objects for further image processing (debug image provides only the moving objects)
  Based on the learning rate parameter the node can used in different modes:
  1. Learning rate 0.0 gives the change in the current frame compared to the very first frame (first frame always acts as the static background model)
  2. Learning rate 0.99 gives the change in the current frame compared to the previous frame (background model is updated immediately)
  3. Intermediate learning rates can be used to update the background model at the required pace
  Also, the background_change_threshold defines the amount of change required for a change event to be published
  See merge request !51
* update internal state handling
* roslaunch tests and update install targets
* add as library
* change object name
* Use initialize method to clear background model to avoid new object creation
* Remove e_trigger mode and switch only based on current_state\_ variable
* [mcr_background_change_detection] Node to detect moving objects in a 2D scene - v1.0
* Contributors: Alex Mitrevski, Argentina Ortega Sainz, Ashok Meenakshi Sundaram, Frederik Hegger, Santosh Thoduka, deebuls, jcmayoral, shehzad ahmed
