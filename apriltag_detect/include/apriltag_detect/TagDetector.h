#include <ros/ros.h>
#include <string>
#include <memory>
#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag16h5.h>
#include <tagStandard52h13.h>
#include <tagStandard41h12.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tagCustom48h12.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <apriltag_detect/graphing.h>

//#define ERROR_THRESHOLD 0.001

using std::string;

class TagDetector{
private:
  apriltag_detector_t *td_;
  apriltag_family_t *tf_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber sub_;
  ros::Publisher pose_publisher_;
  ros::Publisher est_pose_publisher_;
  ros::Publisher tag_detected_publisher_;
  ros::Publisher points_publisher_;
  tf::TransformBroadcaster br_;
  tf::StampedTransform transform_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_image_subscriber_;
  zarray_t *detected_tags_;

  std::string parent_frame_;
  std::string child_frame_;
  std::string tag_family_;
  std::string pose_topic_;
  std::string landing_pad_frame_;

  int tag_id_;
  int tag_threads_;
  double tag_size_;
  double tag_decimate_;
  double tag_blur_;
  int tag_refine_edges_;
  int tag_debug_;

  void detectTag(
    const sensor_msgs::ImageConstPtr& image,
    const sensor_msgs::CameraInfoConstPtr& camera_info);
  static int idComparison(const void* first, const void* second);
  void removeDuplicates();

public:
  TagDetector(int argc, char** argv);
  ~TagDetector();

  void init();
};
