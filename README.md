# Aerial-Robotics
Landing of a UAV on a UGV.

## Download package and bag file
The package can be retrieved from: 

https://github.com/Kripash/Aerial-Robotics

The bag file with vicon data can be retrieved from:

https://drive.google.com/drive/folders/1GyGuS-8kLymNSwk_5a1HJ7Vn80-iR6FM?usp=sharing

## How to get it to work

1. Build the `apriltag_detect` package.
2. Launch the launch file via `roslaunch apriltag_detect apriltag_detect.launch` after sourcing the workspace.
3. Play the downloaded rosbag.

## Expected result

1. Rviz window should open and display the images from the camera, the pose provided by April Tag and the pose provided by the vicon.
2. Once the rosbag finishes playing, wait for about 5 seconds and a new window displaying the pose estimation error graph will appear.

## Topics

- `/detector/error`
  - Only use for evaluation.
  - Error between the estimated pose from the april tag detection and the position provided by the vicon.
    Used for plotting point density graph with different colors showing the magnitude of error of each detection.
- `/detector/estimated_landing_pad`
  - Estimated pose of the landing pad w.r.t. the april tag frame.
- `/detector/graphing_points`
  - Only use for evaluation.
  - 4 points showing where the tag is detected on the camera.
  - Used for plotting point density graph.
- `/detector/pose`
  - Estimated pose of the april tag w.r.t. the landing pad frame computed using the camera.
- `/detector/tag_detection`
  - Images from the camera with a square showing where the tag is detected.
  - Can be viewed using rviz.
