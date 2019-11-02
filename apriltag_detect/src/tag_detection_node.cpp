#include <iostream>
#include <apriltag.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include "../include/TagDetector.h"

int main(int argc, char **argv){
  TagDetector tag_detector{argc, argv, "detect"};
  ros::spin();
  return 0;
}
