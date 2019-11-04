#include <ros/ros.h>
#include <string>
#include <apriltag.h>
#include <tag16h5.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::string;

class TagDetector{
private:
  apriltag_detector_t *td_;
  apriltag_family_t *tf_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  void detectTag(const sensor_msgs::ImageConstPtr& image);

public:
  TagDetector();
  ~TagDetector();

  void init();
};
