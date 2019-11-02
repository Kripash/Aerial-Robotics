#include <ros/console.h>
#include <apriltag.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

class TagDetector{
private:
  apriltag_detector_t *td_;
  apriltag_family_t *tf_;

public:
  TagDetector();
  ~TagDetector();
}
