#include <ros/ros.h>
#include <string>
#include <apriltag.h>
#include <tag16h5.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

using std::string;

class TagDetector{
private:
  apriltag_detector_t *td_;
  apriltag_family_t *tf_;
  ros::Subscriber sub_;

  void detectTag(const sensor_msgs::ImagePtr& image);

public:
  TagDetector(int argc, char **argv, const string& node_name);
  ~TagDetector();

  void init(int argc, char **argv, const string& node_name);
};
