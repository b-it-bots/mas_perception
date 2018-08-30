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
1. Enable dataset_collection parameter in the launch file
2. Set the logdir parameter (optional), by default it is "/tmp/"
3. Start collecting dataset
** rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String "data: 'e_start'"
** rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String "data: 'e_add_cloud_start'"
```

Debug mode:
```
If this is enable in the launch file, the pointcloud will be saved to the logdir.
```