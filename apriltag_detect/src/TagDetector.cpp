#include "../include/TagDetector.h"

TagDetector::TagDetector(int argc, char **argv, const string& node_name):
  td_{apriltag_detector_create()},
  tf_{tag16h5_create()}
{
  apriltag_detector_add_family(td_, tf_);
  init(argc, argv, node_name);
}

TagDetector::~TagDetector(){
  tag16h5_destroy(tf_);
  apriltag_detector_destroy(td_);
}

void TagDetector::init(int argc, char **argv, const string& node_name){
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;
  sub_ = n.subscribe("ptgrey_node/cam/image_raw",1, &TagDetector::detectTag, this);
}

void TagDetector::detectTag(const sensor_msgs::ImagePtr& image){
  zarray_t *detections = nullptr;

  image_u8_t apriltag_image = {image->};

  detections = apriltag_detector_detect(td_, apriltag_image);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    ROS_INFO_STREAM("FOUND! " << i);
  }
}
