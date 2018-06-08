# Scene segmentation

### Usage

Find plane and segment objects
```
rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_find_plane
```


Find plane and segment objects
```
rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_segment

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_stop
```

Subscribe to the following topics:
Object list:
```
/mcr_perception/object_detector/object_list
```

Bounding Boxes (for visualization in Rviz)
```
/mcr_perception/scene_segmentation/bounding_boxes
```

Object labels (for visualization in Rviz)
```
/mcr_perception/scene_segmentation/labels
```

Debug pointcloud (shows filtered input to plane segmentation)
```
/mcr_perception/scene_segmentation/output
```

Object clusters
```
/mcr_perception/scene_segmentation/tabletop_clusters
```

Workspace height:
```
/mcr_perception/scene_segmentation/workspace_height
```


Dataset collection:
```
Setting in launch file:
  * Set dataset_collection to true
  * Set is_classifier_required to true if the corresponding object names needed for the pcd name <br/>
  (e.g. "/tmp/AXIS_1528290814.pcd"), otherwise e.g. "/tmp/atwork_object_1528288932.pcd". <br/>
  This needs mcr_object_recognition_mean_circle (roslaunch mcr_object_recognition_mean_circle object_recognition.launch 
)

How to:

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_collect_dataset

```