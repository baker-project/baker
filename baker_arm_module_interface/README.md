# Baker Arm Module interface

**WARNING** `base_link` changed from `world` to `base_link` in `ipa_arm_planning/config/baker/arm_planning_config.yaml`

## Dependencies

See the `ipa_arm_planning_demo` from the `ipa_manipulation` package. 

## Server

### Actions

* `/take_trashcan` (`moveToAction`): the arm moves and take the trashcan. The given position is where the gripper should be to close his fingers.
Nothing done if the arm has already a trashcan
* `/leave_trashcan` (`moveToAction`): the arm leaves the trashcan on the given position. This position is where the gripper should to open his fingers. Nothing done if the arm doesn't carry a trashcan.
* `/empty_trashcan` (`moveToAction`): the arm moves on the given position (should be on top of the trolley) and empties the trashcan. Nothing done if the arm doesn't carry a full trashcan.
* `/rest_position` (`moveToAction`): the arm moves on his rest position (used when the robot doesn't carry a trashcan). Nothing done if the robot carries a trashcan.
* `/transport_position` (`moveToAction`): the arm moves on his transport position (used when the robot carries a trashcan). Nothing done if the robot doesn't carry a trashcan.
* `/set_joints_values` (`ExecuteTrajectoryAction`): the arm moves to this joints position

### Services
None

### Topics

## Client

To start it:
```
rosrun baker_arm_module_interface baker_arm_client.py
```
All actions can be tested one by one.

## Usage

See the `trashcan_empying_behavior` in the main application.
