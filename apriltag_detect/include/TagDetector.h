#include <ros/ros.h>
#include <string>
#include <memory>
#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag16h5.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "image_geometry/pinhole_camera_model.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>

using std::string;

class TagDetector{
private:
  apriltag_detector_t *td_;
  apriltag_family_t *tf_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster br_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_image_subscriber_;
  zarray_t *detected_tags_;

  void detectTag(
    const sensor_msgs::ImageConstPtr& image,
    const sensor_msgs::CameraInfoConstPtr& camera_info);
  static int idComparison(const void* first, const void* second);
  void removeDuplicates();

public:
  TagDetector();
  ~TagDetector();

  void init();
};
