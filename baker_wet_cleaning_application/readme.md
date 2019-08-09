# Baker Wet Cleaning Application

## Notes

* Testing the **dirt detector** and **the dirt removing behavior**
  * Start the main application (as usual)
  * Run
  ```
  rosrun baker_wet_cleaning_application test_detection_and_removing_dirt.py
  ```
  It will start the detector and if the robot detects something, it executes the dirt removing behavior for all the detected dirts. To test the detector it's possible to slowly move manually the robot, it will start moving once a dirt is detected.

* Using the **fake detector** the fake and the real detectors (they use the same services and topics), a variable is defined in the main `roslaunch` file to select which one to start

* Reproducing the **segfault bug** (worked only on the `weimarerStrasse` map)
  1. Start mira
  2. Close Mira
  3. Open the two `SimulatorPose.xml` and change the robot position to:
  ```xml
  <!--The translational part of the transform-->
  <X>-5.49648197174</X>
  <!--The translational part of the transform-->
  <Y>-5.77961158752</Y>
  <!--The orientation in degrees-->
  <Phi>-53.285074947166564</Phi>
  ```
  4. Start the file `test_bug.py` to move the robot to the bugged area

* Test the cleaning in a given room with a given cleaning method: use the python script `scripts/database_scenario` to select the rooms to clean and the associated method. To understand the options, call `python scripts/database_scenario --help`  **Warning**: this script overwrite the database

* To activate or deactivate a fake detector: edit the probabilities directly in the `fake_detector` python code

* To display the trash detections in Rviz, see the `baker_detections_visualizer`. The work was done **only** for trashcans but could be easily updated for dirt too. The visualizer is started by default with the main roslaunch file

* To use the robotic arm:
  1. Start the main application and Mira
  2. Start the robotic arm (some parameters have to be updated for reality):
    ```
    roslaunch baker_wet_cleaning_application robotic_arm.launch
    ```
    See the `Readme.md` file of the `robotic_arm_module` to understand how to use it.

* Idea to allow the _backward_ option for moves:
    1. Add a boolean to the `move_base.action` into `scitos_common`
    2. If this option is activated, change the task into the `scitos_mira/ScitosDrive` `setTaskAndWaitForTarget` function.
    3. In the Dry Cleaning behavior, ask the robot to move to `last_planned_point_index - 1` position (with the backward allowed) **before** starting again the path follower. This backward option could be used by default for many simple move action (for example to go the room center, go the dirt position…)


## How to use it at Metralab
At Metralabs:

0. **login** with baker
1. `roslaunch baker_wet_cleaning_application basic_application.launch scitos_node_args:="-k192.168.5.1:1234"`   `brush_cleaning_module_interface_args:="-k192.168.5.1:1234"`
2. **Start Rviz** on robot PC
3. [NOT NECESSARY ANYMORE: this is now already started in basic_application.launch]: `rosrun baker_wet_cleaning_application application_wet_cleaning.py`
