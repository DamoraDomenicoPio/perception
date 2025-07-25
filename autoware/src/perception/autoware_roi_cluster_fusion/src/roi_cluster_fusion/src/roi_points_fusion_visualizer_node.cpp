#include "roi_points_fusion/roi_points_fusion_visualizer_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


using std::placeholders::_1;
using std::placeholders::_2;

RoiPointsFusionVisualizerNode::RoiPointsFusionVisualizerNode(const rclcpp::NodeOptions & node_options)
: Node("roi_points_fusion_visualizer_node", node_options),
  image_sub_(this, "/camera/compressed", rclcpp::QoS(10).get_rmw_qos_profile()),
  roi_sub_(this, "/perception/object_recognition/detection/rois0_with_cluster", rclcpp::QoS(10).get_rmw_qos_profile())
{





  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), image_sub_, roi_sub_);
  sync_->registerCallback(std::bind(&RoiPointsFusionVisualizerNode::syncCallback, this, _1, _2));

  visualizer_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/object_recognition/debug/roi_with_cluster_visualizer", 10);



  // cv::Rodrigues(cv::Matx33d(0.0017, 0.0130, 0.9999, -0.999, -0.0099, 0.0019, 0.0099, -0.9999, 0.0130).t(), rvec_);
  rvec_ = (cv::Mat_<double>(3, 3) << -0.014222214037495073, -0.9996505901018471, 0.022280626941377335,
                                     0.009395233010328463, -0.022415498319689253, -0.999704593883494,
                                    0.9998547185589282, -0.014008681026747571, 0.009710748237778843);
  tvec_ = (cv::Mat_<double>(3, 1) << -0.005817362146425229, -0.18076945723824778, -0.02473732132118369);
  camera_matrix_ = (cv::Mat_<double>(3, 3) << 807.4156926905575, 0, 662.5427104951784,
                                              0, 807.8710138048565, 506.99645277784333,
                                              0, 0, 1.0000); // Intrinsic camera matrix (K)
  dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.35493939487565995, 0.12534826481361314, 0.00017675365391964785, -0.0008927317535811412, -0.018643914055372864); // Distorsion coefficients.
}


void RoiPointsFusionVisualizerNode::syncCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image_sub_msg,
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_sub_msg)
{
  cv::Mat image = cv::imdecode(cv::Mat(image_sub_msg->data), cv::IMREAD_COLOR);

  for(const auto &obj : roi_sub_msg->feature_objects) {
    const auto &roi = obj.feature.roi;
    std::vector<cv::Vec3f> acquired_points;

    cv::Scalar random_color(
      static_cast<int>(std::rand() % 256),
      static_cast<int>(std::rand() % 256),
      static_cast<int>(std::rand() % 256)
    );

    cv::rectangle(image, 
                  cv::Point(roi.x_offset, roi.y_offset), 
                  cv::Point(roi.x_offset + roi.width, roi.y_offset + roi.height), 
                  random_color, 2);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(obj.feature.cluster, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(obj.feature.cluster, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(obj.feature.cluster, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      acquired_points.emplace_back(*iter_x, *iter_y, *iter_z);
    }

    if (!acquired_points.empty()) {
      

      std::vector<cv::Point2f> p2D;
      p2D.reserve(acquired_points.size());
      cv::Mat acquired_points_mat(acquired_points);
      cv::projectPoints(acquired_points_mat, rvec_, tvec_, camera_matrix_, dist_coeffs_, p2D);
      for (const auto &pt : p2D) {
        if (pt.x >= 0 && pt.y >= 0 && pt.x < image.cols && pt.y < image.rows) {
          cv::circle(image, pt, 1, random_color, -1);
        }
      }
    }
  }

  sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
    image_sub_msg->header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
  visualizer_pub_->publish(*output_msg);
}





int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoiPointsFusionVisualizerNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

