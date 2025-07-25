// roi_points_fusion_visualizer_node.hpp
#ifndef ROI_POINTS_FUSION_VISUALIZER_NODE_HPP
#define ROI_POINTS_FUSION_VISUALIZER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class RoiPointsFusionVisualizerNode : public rclcpp::Node
{
public:
  explicit RoiPointsFusionVisualizerNode(const rclcpp::NodeOptions & node_options);

private:
  void syncCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image_sub_msg,
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_sub_msg);

  // message_filters subscribers
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> roi_sub_;

  // synchronizer
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CompressedImage,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature
    > MySyncPolicy;
  std::shared_ptr< message_filters::Synchronizer<MySyncPolicy> > sync_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualizer_pub_;

  cv::Mat rvec_, tvec_, camera_matrix_, dist_coeffs_;

  const float IMG_WIDTH = 1440.0f; // Camera image width
  const float IMG_HEIGHT = 1080.0f; // Camera image height
};

#endif  // ROI_POINTS_FUSION_VISUALIZER_NODE_HPP
