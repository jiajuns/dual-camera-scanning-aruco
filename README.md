aruco_ros
=========

Software package and ROS2 wrappers of the [Aruco][1] Augmented Reality marker detector library.
for apriltag detection

### Images
```bash 
ros2 launch realsense2_camera rs_launch.py     enable_infra1:=true     enable_infra2:=true     enable_color:=false     enable_depth:=false     depth_module.infra_profile:=1280x800x30     depth_module.emitter_on_off:=false
```


### Calibs

```bash
python3 ~/ros2_ws/src/cube_pose_fuser/scripts/camera_info_override.py --ros-args     -p camera_name:=infra1     -p camera_info_url:=file:///home/nvidia/ros2_ws/src/cube_pose_fuser/calib_data/infra1.yaml     -p image_topic:=/camera/camera/infra1/image_rect_raw     -p camera_info_topic:=/camera/camera/infra1/camera_info_calib
```

 * Object pose estimation
 * Visual servoing: track object and hand at the same time

<img align="right" src="https://raw.github.com/pal-robotics/aruco_ros/master/aruco_ros/etc/reem_gazebo_floating_marker_world.png"/>


### Scan dual_camera aruco

```bash
source install/setup.bash
ros2 launch aruco_ros apriltag_stereo_launch.py
```

### Get data 
```bash
ros2 topic echo /detections 
ros2 run rqt_image_view rqt_image_view
```
