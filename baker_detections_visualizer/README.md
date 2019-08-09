# Baker Detections Visualizer

This module displays in Rviz both dirt & trashcan detections in the `map` base.

Expects:
 * `cob_object_detection_msgs/Detection.msg` for trashcan.
 * `TODO rmb-ds` for dirt

## Trashcan visualizer

The trashcan is visualized as: 
1. The **gripping pose** (link displayed) (`detection.pose`)
2. The **trashcan** as a **cylinder** of height `detection.bounding_box_lwh.z` and of radius `detection.bounding_box_lwh.x` such as the gripping position is tangent to the cylinder.



## Usage


```
roslaunch
```
