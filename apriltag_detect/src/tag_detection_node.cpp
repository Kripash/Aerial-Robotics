#include <iostream>
#include <apriltag.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

void detectTag(const sensor_msgs::ImagePtr& image){
  zarray_t *detections = nullptr;

  image_u8_t apriltag_image = {image->Image_};

  detections = apriltag_detector_detect(td, apriltag_image);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    ROS_INFO_STREAM("FOUND! " << i);
  }
}

// void cleanup(apriltag_detector_t * td, apriltag_family_t *tf){
//   tag16h5_destroy(tf);
//   apriltag_detector_destroy(td);
// }

int main(int argc, char **argv){
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_family_t *tf = tag16h5_create();
  apriltag_detector_add_family(td, tf);

  ros::init(argc, argv, "detector");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscriber("ptgrey_node/cam/image_raw", 1, detectTag);
  ros::spin();
  return 0;
}
