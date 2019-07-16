# Baker Arm Module interface

## Server

### Actions

* `/take_trashcan` (`moveToAction`): the arm moves and take the trashcan. The given position is where the gripper should be to close his fingers
* `/leaveTrashcan` (`moveToAction`): the arm leaves the trashcan on the given position. This position is where the gripper should to open his fingers
* `/empty_trashcan` (`moveToAction`): the arm moves on the given position (should be on top of the trolley) and empties the trashcan
* `/rest_position` (`moveToAction`): the arm moves on his rest position (used when the robot doesn't carry a trashcan)
* `/transport_position` (`moveToAction`): the arm moves on his transport position (used when the robot carries a trashcan)
* `/set_joints_values` (`ExecuteTrajectoryAction`): the arm moves to this joints position


### Services
None

### Topics
None

## Client

To start it:
```
rosrun baker_arm_module_interface baker_arm_client.py
```
All actions can be tested one by one.
