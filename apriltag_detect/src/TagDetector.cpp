#include "../include/TagDetector.h"

TagDetector::TagDetector(int argc, char** argv):
  td_{apriltag_detector_create()},
  nh_{},
  it_{nh_},
  detected_tags_{nullptr}
{
  node_ = new ros::NodeHandle("~");
  node_ -> getParam("tag_size", tag_size_);
  node_ -> getParam("parent_frame", parent_frame_);
  node_ -> getParam("child_frame", child_frame_);
  node_ -> getParam("family", tag_family_);
  node_ -> getParam("tag_id", tag_id_);
  node_ -> getParam("pose_topic", pose_topic_);
  node_ -> getParam("landing_pad_frame", landing_pad_frame_);

  if (tag_family_ == "tag36h11")
  {
    tf_ = tag36h11_create();
  }
  else if (tag_family_ == "tag25h9")
  {
    tf_ = tag25h9_create();
  }
  else if (tag_family_ == "tag16h5")
  {
    tf_ = tag16h5_create();
  }
  else if (tag_family_ == "tagCustom48h12")
  {
    tf_ = tagCustom48h12_create();
  }
  else if (tag_family_ == "tagStandard52h13")
  {
    tf_ = tagStandard52h13_create();
  }  
  else if (tag_family_ == "tagStandard41h12")
  {
    tf_ = tagStandard41h12_create();
  }
  else
  {
    ROS_WARN("Invalid tag family specified! Aborting");
    exit(1);
  }  
  init();
}

TagDetector::~TagDetector(){
  tag16h5_destroy(tf_);
  apriltag_detector_destroy(td_);
  delete node_;
  node_ = nullptr;
}

void TagDetector::init(){
  apriltag_detector_add_family(td_, tf_);
  camera_image_subscriber_ = it_.subscribeCamera(
    "image_rect", 1, &TagDetector::detectTag, this);
  pose_publisher_ = node_ -> advertise<geometry_msgs::PoseStamped>(pose_topic_, 100);
}

void TagDetector::detectTag(
  const sensor_msgs::ImageConstPtr& image,
  const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  cv_bridge::CvImagePtr cv_ptr;
  tf::Transform transform;
  tf::Quaternion q;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray_image = cv_ptr->image;
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);
  double fx = camera_model.fx();
  double fy = camera_model.fy();
  double cx = camera_model.cx();
  double cy = camera_model.cy();

  image_u8_t apriltag_image = {
    .width = gray_image.cols,
    .height = gray_image.rows,
    .stride = gray_image.cols,
    .buf = gray_image.data
  };

  if (detected_tags_){
    apriltag_detections_destroy(detected_tags_);
    detected_tags_ = nullptr;
  }
  detected_tags_ = apriltag_detector_detect(td_, &apriltag_image);
  removeDuplicates();

  for (int i = 0; i < zarray_size(detected_tags_); i++) {
    apriltag_detection_t *det;
    zarray_get(detected_tags_, i, &det);
    if (det->id == tag_id_){
      // ROS_INFO_STREAM("FOUND!");
      apriltag_detection_info_t info;
      apriltag_pose_t pose;
      info.det = det;
      info.tagsize = tag_size_;
      info.fx = fx;
      info.fy = fy;
      info.cx = cx;
      info.cy = cy;
      double err = estimate_tag_pose(&info, &pose);

      transform.setOrigin(tf::Vector3(
        matd_get_scalar(&pose.t[0]),
        matd_get_scalar(&pose.t[1]),
        matd_get_scalar(&pose.t[2])
      ));
      q.setRPY(
        matd_get_scalar(&pose.R[0]),
        matd_get_scalar(&pose.R[1]),
        matd_get_scalar(&pose.R[2])
      );
      transform.setRotation(q);
      br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_, child_frame_));
        
      geometry_msgs::PoseStamped P;
      P.header.frame_id = landing_pad_frame_;
      P.header.stamp = ros::Time::now();
      P.pose.position.x = matd_get_scalar(&pose.t[0]);
      P.pose.position.y = matd_get_scalar(&pose.t[1]);
      P.pose.position.z = matd_get_scalar(&pose.t[2]);

      pose_publisher_.publish(P);
   
    }
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
