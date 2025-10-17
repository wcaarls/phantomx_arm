# phantomx_arm
Phantomx pincher arm package for ROS2

This package is based on the original phantomx_arm package for ROS1 by Michael Farnsworth
(cdrwolfe). It has been updated for ROS2, and now uses a ros2_control
interface to the arm. Only basic arm and gripper movement is supported.

To launch the demo, just use

```
ros2 launch phantomx_arm_bringup planning_demo.launch.py
```

