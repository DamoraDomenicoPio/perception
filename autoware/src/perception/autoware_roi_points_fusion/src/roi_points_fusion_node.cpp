#include "autoware_roi_points_fusion/roi_points_fusion_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
using std::string;

using std::placeholders::_1;
using std::placeholders::_2;

RoiPointsFusionNode::RoiPointsFusionNode(const rclcpp::NodeOptions & node_options)
: Node("roi_points_fusion_node", node_options),
  points_sub_(this, "~/in/lidar", rclcpp::QoS(10).get_rmw_qos_profile()),
  roi_sub_(this, "~/in/rois", rclcpp::QoS(10).get_rmw_qos_profile())
{
  min_y = this->declare_parameter<float>("lidar.min_y");
  max_y = this->declare_parameter<float>("lidar.max_y");
  min_z = this->declare_parameter<float>("lidar.min_z");
  max_z = this->declare_parameter<float>("lidar.max_z");
  IMG_WIDTH = this->declare_parameter<float>("camera.width", IMG_WIDTH);
  IMG_HEIGHT = this->declare_parameter<float>("camera.height", IMG_HEIGHT);

  std::vector<double> R = this->declare_parameter<std::vector<double>>("calibration.R");
  std::vector<double> tvec = this->declare_parameter<std::vector<double>>("calibration.tvec");
  std::vector<double> camera_matrix = this->declare_parameter<std::vector<double>>("calibration.camera_matrix");
  std::vector<double> dist_coeffs = this->declare_parameter<std::vector<double>>("calibration.dist_coeffs");
  R_ = (cv::Mat_<double>(3, 3) << R[0], R[1], R[2],
                                   R[3], R[4], R[5],
                                   R[6], R[7], R[8]);
  tvec_ = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
  camera_matrix_ = (cv::Mat_<double>(3, 3) << camera_matrix[0], camera_matrix[1], camera_matrix[2],
                                               camera_matrix[3], camera_matrix[4], camera_matrix[5],
                                               camera_matrix[6], camera_matrix[7], camera_matrix[8]);
  dist_coeffs_ = (cv::Mat_<double>(1, 5) << dist_coeffs[0], dist_coeffs[1], dist_coeffs[2], dist_coeffs[3], dist_coeffs[4]);

  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), points_sub_, roi_sub_);
  sync_->registerCallback(std::bind(&RoiPointsFusionNode::syncCallback, this, _1, _2));

  detection_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>("~/out/rois", 10);



  // cv::Rodrigues(cv::Matx33d(0.0017, 0.0130, 0.9999, -0.999, -0.0099, 0.0019, 0.0099, -0.9999, 0.0130).t(), rvec_);
  // R = (cv::Mat_<double>(3, 3) << -0.014222214037495073, -0.9996505901018471, 0.022280626941377335,
  //                                    0.009395233010328463, -0.022415498319689253, -0.999704593883494,
  //                                   0.9998547185589282, -0.014008681026747571, 0.009710748237778843);
  // tvec_ = (cv::Mat_<double>(3, 1) << -0.005817362146425229, -0.18076945723824778, -0.02473732132118369);
  // camera_matrix_ = (cv::Mat_<double>(3, 3) << 807.4156926905575, 0, 662.5427104951784,
  //                                             0, 807.8710138048565, 506.99645277784333,
  //                                             0, 0, 1.0000); // Intrinsic camera matrix (K)
  // dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.35493939487565995, 0.12534826481361314, 0.00017675365391964785, -0.0008927317535811412, -0.018643914055372864); // Distorsion coefficients.
}


void RoiPointsFusionNode::syncCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_sub_msg,
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_msg)
{
  std::vector<cv::Vec3f> acquired_points;
  std::vector<float> points_intensity;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_sub_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*points_sub_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*points_sub_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*points_sub_msg, "intensity");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    if (*iter_x >= 0 &&
        *iter_y >= min_y && *iter_y <= max_y &&
        *iter_z >= min_z && *iter_z <= max_z) {
      acquired_points.emplace_back(*iter_x, *iter_y, *iter_z);
      points_intensity.push_back(*iter_intensity);
    }
  }


  tier4_perception_msgs::msg::DetectedObjectsWithFeature fused_msg = points_roi_fusion(roi_msg, acquired_points);
  detection_pub_->publish(fused_msg);
  

}




tier4_perception_msgs::msg::DetectedObjectsWithFeature
RoiPointsFusionNode::points_roi_fusion(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_msg,
  const std::vector<cv::Vec3f> &acquired_points)
{
  
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature fused_msg;
  fused_msg.header = roi_msg->header;
  fused_msg.feature_objects = roi_msg->feature_objects;

  std::vector<sensor_msgs::msg::PointCloud2> clouds(roi_msg->feature_objects.size());

  for (size_t j = 0; j < clouds.size(); ++j) {
    auto &cloud = clouds[j];
    cloud.header = roi_msg->header;
    cloud.height = 1;
    cloud.width = 0; 
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.point_step = sizeof(float) * 3;  // x,y,z
    cloud.row_step = 0;
    // fields: x, y, z float32
    sensor_msgs::msg::PointField f;
    f.count = 1;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.offset = 0; f.name = "x"; clouds[j].fields.push_back(f);
    f.offset = 4; f.name = "y"; clouds[j].fields.push_back(f);
    f.offset = 8; f.name = "z"; clouds[j].fields.push_back(f);
  }

  std::vector<cv::Point2f> p2D;
  cv::Mat acquired_points_mat(acquired_points);
  cv::projectPoints(acquired_points_mat, R_, tvec_, camera_matrix_, dist_coeffs_, p2D);

  for (size_t i = 0; i < p2D.size(); ++i){
      const float x = p2D[i].x;
      const float y = p2D[i].y;
      if (x < 0 || y < 0 || x > IMG_WIDTH || y > IMG_HEIGHT) {
          continue;
      }

      const cv::Vec3f &acquired_point = acquired_points[i];

      for(size_t j = 0; j < roi_msg->feature_objects.size(); ++j) {
          const auto & roi = roi_msg->feature_objects[j].feature.roi;
          if (x >= roi.x_offset && x < (roi.x_offset + roi.width) &&
              y >= roi.y_offset && y < (roi.y_offset + roi.height)) {
              auto &cloud = clouds[j];
              float buf[3] = {acquired_point[0], acquired_point[1], acquired_point[2]};
              const auto bytes = reinterpret_cast<uint8_t*>(buf);
              cloud.data.insert(cloud.data.end(), bytes, bytes + sizeof(buf));
              cloud.width++;
          }
      }
  }

  for (size_t j = 0; j < clouds.size(); ++j) {
    auto &cloud = clouds[j];
    cloud.row_step = cloud.point_step * cloud.width;

    fused_msg.feature_objects[j].feature.cluster = cloud;
  }


  return fused_msg;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoiPointsFusionNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

