# Baker Arm Module interface

## Server

### Actions

* `/take_trashcan` (`moveToAction`): the arm moves and take the trashcan. The given position is where the gripper should be to close his fingers.
Nothing done if the arm has already a trashcan
* `/leave_trashcan` (`moveToAction`): the arm leaves the trashcan on the given position. This position is where the gripper should to open his fingers. Nothing done if the arm doesn't carry a trashcan.
* `/empty_trashcan` (`moveToAction`): the arm moves on the given position (should be on top of the trolley) and empties the trashcan. Nothing done if the arm doesn't carry a full trashcan.
* `/rest_position` (`moveToAction`): the arm moves on his rest position (used when the robot doesn't carry a trashcan). Nothing done if the robot carries a trashcan.
* `/transport_position` (`moveToAction`): the arm moves on his transport position (used when the robot carries a trashcan). Nothing done if the robot doesn't carry a trashcan.
* `/set_joints_values` (`ExecuteTrajectoryAction`): the arm moves to this joints position
* `/accessibility_checker` (`moveToAction`): returns True if and only if the arm can access the target position from this robot position.

### Services
None

### Topics
* `/status`:
  * _0_: No Trashcan
  * _1_: Empty Trashcan
  * _2_: Full Trashcan

## Client

To start it:
```
rosrun baker_arm_module_interface baker_arm_client.py
```
All actions can be tested one by one.

## Usage

1. Initialize the robotic arm position before using the robot (call `/rest_position`)
