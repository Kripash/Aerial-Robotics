#include "../include/TagDetector.h"

TagDetector::TagDetector():
  td_{apriltag_detector_create()},
  tf_{tag16h5_create()},
  nh_{},
  detected_tags_{nullptr}
{
  apriltag_detector_add_family(td_, tf_);
  init();
}

TagDetector::~TagDetector(){
  tag16h5_destroy(tf_);
  apriltag_detector_destroy(td_);
}

void TagDetector::init(){
  sub_ = nh_.subscribe("ptgrey_node/cam/image_raw",1, &TagDetector::detectTag, this);
}

void TagDetector::detectTag(const sensor_msgs::ImageConstPtr& image){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
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

  detected_tags_ = apriltag_detector_detect(td_, &apriltag_image);
  // for (int i = 0; i < zarray_size(detections); i++) {
  //   apriltag_detection_t *det;
  //   zarray_get(detections, i, &det);
  //
  //   ROS_INFO_STREAM("FOUND! " << i);
  // }
  removeDuplicates();
  ROS_INFO_STREAM("FOUND! " << zarray_size(detected_tags_));
  for (int i = 0; i < zarray_size(detected_tags_); i++) {
    apriltag_detection_t *det;
    zarray_get(detected_tags_, i, &det);

    ROS_INFO_STREAM("ID: " << det->id);
  }
}

int TagDetector::idComparison(const void* first, const void* second)
{
  int id1 = ((apriltag_detection_t*) first)->id;
  int id2 = ((apriltag_detection_t*) second)->id;
  return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void TagDetector::removeDuplicates(){
  zarray_sort(detected_tags_, &idComparison);
  int count = 0;
  bool duplicate_detected = false;
  while(true){
    if (count > zarray_size(detected_tags_)-1){
      return;
    }
    apriltag_detection_t *detected_tag;
    zarray_get(detected_tags_, count, &detected_tag);
    int id_current = detected_tag->id;
    int id_next = -1;
    if (count < zarray_size(detected_tags_)-1){
      zarray_get(detected_tags_, count+1, &detected_tag);
      id_next = detected_tag->id;
    }
    if (id_current == id_next || (id_current != id_next && duplicate_detected)){
      duplicate_detected = true;
      int shuffle = 0;
      zarray_remove_index(detected_tags_, count, shuffle);
      if (id_current != id_next){
        duplicate_detected = false;
      }
      continue;
    } else {
      ++count;
    }
  }
}
