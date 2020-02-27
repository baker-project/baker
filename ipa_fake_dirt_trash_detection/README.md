# Ipa Fake Dirt Trash Detection

## Usage

```
roslaunch ipa_fake_dirt_trash_detection ipa_fake_dirt_trash_detector.launch
  dirt_detection:=[false / true]
  trashcan_detection:=[false / true]
```

The detector can use absolute coordinates or random ones. It has to be edited directly into the code.

## Services

* For the **dirt detector** _(compatible with ipa_dirt_detection)_:
```
/dirt_detection_server_preprocessing/activate_detection Trigger
/dirt_detection_server_preprocessing/deactivate_detection Trigger
```

* For the **trashcan detector**:
```
/trash_detector/activate_detection Trigger
/trash_detection/deactivate_detection Trigger
```

## Topics
* **Always**, if fake detector is started
```
/camera/depth_registered/points sensor_msgs::PointCloud2
```
It allows to execute `scitos_mira::publishCameraPosition` and therefore send on `tf` the camera transformation.

* For the **dirt detector** _(compatible with ipa_dirt_detection)_:
```
/dirt_detection_server_preprocessing/dirt_detector_topic cob_object_detection_msgs::DetectionArray
```

* For the **trashcan detector**:
```
trash_detector_topic cob_object_detection_msgs::DetectionArray
```

Detections are published on `frame_id="camera2_optical_frame"`

## Visualization in RViz

**only for trashcan** (could be easily done for dirt) - see `baker_detections_visualizer` module
