#include <iostream>
#include <apriltag.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include "apriltag_detect/TagDetector.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "tag_detector");
  TagDetector tag_detector(argc, argv);
  ros::spin();
  return 0;
}
