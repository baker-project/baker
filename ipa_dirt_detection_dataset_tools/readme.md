# Dirt Detection Dataset Tools

## Data Recording
For data recording with an Asus Xtion Pro Live run the following commands in different terminals:
```
roslaunch openni2_launch openni2.launch
rqt --> open dynamic reconfigure and select camera/driver --> there change color_mode to 1 (SXGA/30Hz)
roslaunch ipa_dirt_detection_dataset_tools record_dataset_rgb.launch
```
### Clean Ground Floor Images
- Take normal images of the clean ground floor which just contain the floor and nothing else (no walls, no objects, no dirt).
- The camera points to the ground at 90 deg angle and the pictures are taken from different views (different locations, illumination conditions, camera rotations).

### Dirt and Objects
- The dirt spots and objects are placed on uniformly colored background. Ideally, a rectangular sheet of (colored) paper is used a background. It should have some contrast to the background of the sheet. The edges of the rectangular sheet should be aligned in parallel with the camera image.
- The background color should be strongly contrasting to the object's colors (i.e. segmentation will later remove all parts with the background color from the scene).
- The dirt and object images belonging to the same class (e.g. Food dirt, Paper dirt, Pen objects, Notebook objetcs) should be stored in the same folder
and the folder should be named with the class name.

## Image Segmentation
For segmenting just the dirt spots and objects from the respective recordings, start:
```
roslaunch ipa_dirt_detection_dataset_tools simple_segmentation.launch
```

## Image synthesis
For blending realistic scene images from clean floor pictures, dirt samples, object samples, and additional shadows and illumination sources, run the following program:
```
roslaunch ipa_dirt_detection_dataset_tools image_blender.launch
```
