// roi_cluster_fusion_node.hpp
#ifndef ROI_CLUSTER_FUSION_NODE_HPP
#define ROI_CLUSTER_FUSION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

class RoiClusterFusionNode : public rclcpp::Node
{
public:
  explicit RoiClusterFusionNode(const rclcpp::NodeOptions & node_options);

private:
  void syncCallback(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & cluster_sub_msg,
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_msg,
    /* *** */
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

  std::vector<cv::Vec3f> remap_points(const std::vector<cv::Vec3f> &acquired_points);

  // message_filters subscribers
  message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> cluster_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> roi_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  
  // synchronizer
  typedef message_filters::sync_policies::ApproximateTime<
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    /* *** */
    sensor_msgs::msg::Image> MySyncPolicy;
  std::shared_ptr< message_filters::Synchronizer<MySyncPolicy> > sync_;

  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  cv::Mat rvec_, tvec_, camera_matrix_, dist_coeffs_;
  float min_y = -200.0f; // Minimum y-coordinate for the bounding box
  float max_y = 200.0f;  // Maximum y-coordinate for the bounding
  float min_z = -10.0f; // Minimum z-coordinate for the bounding box
  float max_z = 10.0f; // Maximum z-coordinate for the bounding box
  // double SCALE_FACTOR = 1.7778;
  double SCALE_FACTOR = 1.3333333;
  bool debug_ = false;
};

#endif  // ROI_CLUSTER_FUSION_NODE_HPP
