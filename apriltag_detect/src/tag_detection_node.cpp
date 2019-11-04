#include <iostream>
#include <apriltag.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include "../include/TagDetector.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "detect");
  TagDetector tag_detector{};
  ros::spin();
  return 0;
}
