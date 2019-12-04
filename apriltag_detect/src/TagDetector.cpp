#include "apriltag_detect/TagDetector.h"

TagDetector::TagDetector(int argc, char** argv):
  td_{apriltag_detector_create()},
  nh_{new ros::NodeHandle{"~"}},
  it_{ros::NodeHandle{}}, //needs nodehandle for itself (it uses std::move).
  detected_tags_{nullptr}
{
  nh_->getParam("tag_size", tag_size_);
  nh_->getParam("parent_frame", parent_frame_);
  nh_->getParam("child_frame", child_frame_);
  nh_->getParam("family", tag_family_);
  nh_->getParam("tag_id", tag_id_);
  nh_->getParam("pose_topic", pose_topic_);
  nh_->getParam("landing_pad_frame", landing_pad_frame_);
  nh_->getParam("tag_threads", tag_threads_);
  nh_->getParam("tag_decimate", tag_decimate_);
  nh_->getParam("tag_blur", tag_blur_);
  nh_->getParam("debug", tag_debug_);
  nh_->getParam("refine_edges", tag_refine_edges_);


  td_->quad_decimate = static_cast<float>(tag_decimate_);
  td_->quad_sigma = static_cast<float>(tag_blur_);
  td_->nthreads = tag_threads_;
  td_->debug = tag_debug_;
  td_->refine_edges = tag_refine_edges_;

  tf::TransformListener listener;
  try{
    listener.waitForTransform(landing_pad_frame_, parent_frame_, ros::Time::now(), ros::Duration(1.0));
    listener.lookupTransform(landing_pad_frame_, parent_frame_, ros::Time::now(), transform_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

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
    ROS_WARN("Invalid tag family! Exiting!");
    exit(1);
  }
  init();
}

TagDetector::~TagDetector(){
  apriltag_detector_destroy(td_);
  if (tag_family_ == "tag36h11")
  {
    tag36h11_destroy(tf_);
  }
  else if (tag_family_ == "tag25h9")
  {
    tag25h9_destroy(tf_);
  }
  else if (tag_family_ == "tag16h5")
  {
    tag16h5_destroy(tf_);
  }
  else if (tag_family_ == "tagCustom48h12")
  {
    tagCustom48h12_destroy(tf_);
  }
  else if (tag_family_ == "tagStandard52h13")
  {
    tagStandard52h13_destroy(tf_);
  }
  else if (tag_family_ == "tagStandard41h12")
  {
    tagStandard41h12_destroy(tf_);
  }

  nh_.reset();
}

void TagDetector::init(){
  apriltag_detector_add_family(td_, tf_);
  camera_image_subscriber_ = it_.subscribeCamera(
    "image_rect", 1, &TagDetector::detectTag, this);
  pose_publisher_ = nh_->advertise<geometry_msgs::PoseStamped>(pose_topic_, 100);
  est_pose_publisher_ = nh_->advertise<geometry_msgs::PoseStamped>( std::string("estimated_") + landing_pad_frame_, 100);
  tag_detected_publisher_ = nh_->advertise<sensor_msgs::Image>("tag_detection", 1);
  points_publisher_= nh_->advertise<apriltag_detect::graphing>("graphing_points",1);
}

void TagDetector::detectTag(
  const sensor_msgs::ImageConstPtr& image,
  const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  cv_bridge::CvImagePtr cv_ptr;
  tf::Transform transform;
  tf::Quaternion q;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
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
      apriltag_detection_info_t info;
      apriltag_pose_t pose;

      info.det = det;
      info.tagsize = tag_size_;
      info.fx = fx;
      info.fy = fy;
      info.cx = cx;
      info.cy = cy;
      double err = estimate_tag_pose(&info, &pose);

      cv::line(cv_ptr->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Scalar(255, 0, 0));

      cv::line(cv_ptr->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(255, 0, 0));

      cv::line(cv_ptr->image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Scalar(255, 0, 0));

      cv::line(cv_ptr->image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(255, 0, 0));

      apriltag_detect::graphing points;
      points.point1_x = static_cast<uint32_t>(det->p[0][0]);
      points.point1_y = static_cast<uint32_t>(det->p[0][1]);

      points.point2_x = static_cast<uint32_t>(det->p[1][0]);
      points.point2_y = static_cast<uint32_t>(det->p[1][1]);

      points.point3_x = static_cast<uint32_t>(det->p[2][0]);
      points.point3_y = static_cast<uint32_t>(det->p[2][1]);

      points.point4_x = static_cast<uint32_t>(det->p[3][0]);
      points.point4_y = static_cast<uint32_t>(det->p[3][1]);

      points_publisher_.publish(points);

      #ifdef ERROR_THRESHOLD
      if(err < ERROR_THRESHOLD)
      #endif
      {
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

        q.setRPY(0 ,0 ,0);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_, child_frame_ + std::string("_body_frame")));

        geometry_msgs::PoseStamped P;
        P.header.frame_id = landing_pad_frame_;
        P.header.stamp = ros::Time::now();
        P.pose.position.x = matd_get_scalar(&pose.t[0]) + transform_.getOrigin().x();
        P.pose.position.y = matd_get_scalar(&pose.t[1]) + transform_.getOrigin().y();
        P.pose.position.z = matd_get_scalar(&pose.t[2]) + transform_.getOrigin().z();

        pose_publisher_.publish(P);

        geometry_msgs::PoseStamped est_p;
        est_p.header.frame_id = child_frame_ + std::string("_body_frame");
        est_p.header.stamp = ros::Time::now();
        est_p.pose.position.x = (matd_get_scalar(&pose.t[0]) * -1) - transform_.getOrigin().x();
        est_p.pose.position.y = (matd_get_scalar(&pose.t[1]) * -1) - transform_.getOrigin().y();
        est_p.pose.position.z = (matd_get_scalar(&pose.t[2]) * -1) - transform_.getOrigin().z();

        est_pose_publisher_.publish(est_p);
      }
    }
  }
  tag_detected_publisher_.publish(cv_ptr->toImageMsg());
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
