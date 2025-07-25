// roi_points_fusion_node.hpp
#ifndef ROI_POINTS_FUSION_NODE_HPP
#define ROI_POINTS_FUSION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

class RoiPointsFusionNode : public rclcpp::Node
{
public:
  explicit RoiPointsFusionNode(const rclcpp::NodeOptions & node_options);

private:
  void syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_sub_msg,
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_msg);

  // std::vector<cv::Vec3f> remap_points(const std::vector<cv::Vec3f> &acquired_points);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature points_roi_fusion(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_msg,
    const std::vector<cv::Vec3f> & acquired_points);

  // message_filters subscribers
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> points_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> roi_sub_;

  // synchronizer
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature
    > MySyncPolicy;
  std::shared_ptr< message_filters::Synchronizer<MySyncPolicy> > sync_;

  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr detection_pub_;


  cv::Mat R_, tvec_, camera_matrix_, dist_coeffs_;
  float min_y = -200.0f; // Minimum y-coordinate for the bounding box
  float max_y = 200.0f;  // Maximum y-coordinate for the bounding
  float min_z = -10.0f; // Minimum z-coordinate for the bounding box
  float max_z = 10.0f; // Maximum z-coordinate for the bounding box

  const float IMG_WIDTH = 1440.0f; // Camera image width
  const float IMG_HEIGHT = 1080.0f; // Camera image height
};

#endif  // ROI_POINTS_FUSION_NODE_HPP
