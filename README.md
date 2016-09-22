# SARAFun Sensor pose calibration
This ROS package is proposed in the context of the European project [SARAFun](http://h2020sarafun.eu/). It provides a method for estimating the pose of the static force sensor used in the project, with respect to a camera. But it could be extended for estimating the pose of any static object with respect to a given camera.

The pose calibration relies on a tag marker localized using http://wiki.ros.org/ar_track_alvar.

# Usage
The pose of the optoforce device wrt to the marker is defined in the launch file display_optoforce.launch

The procedure for launching the calibration is: 
```bash
roscore
rosrun usb_cam usb_cam_node
roslaunch roslaunch sarafun_optoforce_calibration launch_calibration.launch
```
To get the pose of the optoforce marker wrt to a reference frame (right now the world):
```bash
rosservice call /optoforce_calibrate  "{}"
```

So far the calibration is done using a usb camera. If using another camera, the launch of the ar_track_alvar node should be adjusted.

# Camera Calibration
To calibrate the camera (if needed), have a look at http://wiki.ros.org/camera_calibration
Assuming a 10x7 chessboard calibration pattern, the command would be:
```bash
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.108 image:=/my_camera/image camera:=/my_camera
```
