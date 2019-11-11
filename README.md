# Aerial-Robotics
Landing of a UAV on a UGV.

## How to get it to work

### Installs

If you do not have `catkin tools` please install from this website: 

​	https://catkin-tools.readthedocs.io/en/latest/installing.html

### Setup

1. Source ROS:

   ```shell
   source /opt/ros/melodic/setup.bash
   ```

2. Create a catkin workspace.

   ```shell
   mkdir landingpad
   cd landingpad
   mkdir src
   catkin init
   ```

3. Copy the `apriltag_detect` package into your catkin workspace's source space. Your catkin workspace should look like this:

   ```shell
   landingpad/
   └── src
       └── apriltag_detect
   ```

4. Build your package via the following command:

   ```shell
   catkin build
   ```

5. Source your catkin workspace.

   ```shell
   source devel/setup.bash
   ```

6. Run the program.

   ```shell
   roslaunch apriltag_detect apriltag_detect.launch
   ```

7. Run the `arl_april.bag`, this bagfile must be downloaded from the shared google drive link:

   ​	https://drive.google.com/drive/folders/1eDRCfzJd5wJxOZvxRea8Wdhmve_Y3qAg?usp=sharing

   ```shell
   rosbag play arl_april.bag
   ```

Rviz should be displaying the tag now.