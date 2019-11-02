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
  sub_ = nh_.subscribe("ptgrey_node/cam/image_raw",1, &TagDetector::detectTag, this);
}

void TagDetector::detectTag(const sensor_msgs::ImageConstPtr& image){
  zarray_t *detections = nullptr;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray_image = cv_ptr->image;

  image_u8_t apriltag_image = {
    .width = gray_image.cols,
    .height = gray_image.rows,
    .stride = gray_image.cols,
    .buf = gray_image.data
  };

  detections = apriltag_detector_detect(td_, apriltag_image);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    ROS_INFO_STREAM("FOUND! " << i);
  }
}
