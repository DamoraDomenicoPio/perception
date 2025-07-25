#include "roi_cluster_fusion/roi_cluster_fusion_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

RoiClusterFusionNode::RoiClusterFusionNode(const rclcpp::NodeOptions & node_options)
: Node("roi_cluster_fusion_node", node_options),
  cluster_sub_(this, "/clusters", rclcpp::QoS(10).get_rmw_qos_profile()),
  roi_sub_(this, "/perception/object_recognition/detection/rois0", rclcpp::QoS(10).get_rmw_qos_profile()),
  image_sub_(this, "/perception/object_recognition/debug/image_visualizer", rclcpp::QoS(10).get_rmw_qos_profile())
{





  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(10), cluster_sub_, roi_sub_, image_sub_);
  sync_->registerCallback(
    std::bind(&RoiClusterFusionNode::syncCallback, this, _1, _2, _3));

  detection_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>("fused_topic", 10);
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);



  rvec_ = (cv::Mat_<double>(3, 3) << -0.014222214037495073, -0.9996505901018471, 0.022280626941377335,
                                     0.009395233010328463, -0.022415498319689253, -0.999704593883494,
                                    0.9998547185589282, -0.014008681026747571, 0.009710748237778843);
  tvec_ = (cv::Mat_<double>(3, 1) << -0.005817362146425229, -0.18076945723824778, -0.02473732132118369);
  camera_matrix_ = (cv::Mat_<double>(3, 3) << 807.4156926905575, 0, 662.5427104951784,
                                              0, 807.8710138048565, 506.99645277784333,
                                              0, 0, 1.0000); // Intrinsic camera matrix (K)
  dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.35493939487565995, 0.12534826481361314, 0.00017675365391964785, -0.0008927317535811412, -0.018643914055372864); // Distorsion coefficients.
  debug_ = true;
}





void RoiClusterFusionNode::syncCallback(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & cluster_sub_msg,
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_msg,
  /* *** */
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image;

    RCLCPP_INFO(this->get_logger(), "Length of feature objects: %zu", cluster_sub_msg->feature_objects.size());

    for (const auto &cluster : cluster_sub_msg->feature_objects) {
      // const sensor_msgs::msg::PointCloud2 & pcl_cluster = cluster.feature.cluster;
      const sensor_msgs::msg::PointCloud2 & pcl_cluster = cluster.feature.cluster;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(pcl_cluster, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(pcl_cluster, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(pcl_cluster, "z");

      std::vector<cv::Vec3f> acquired_points;
      std::vector<cv::Vec3f> new_points;

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        acquired_points.emplace_back(x, y, z);
      }
      // Remap the acquired points to the camera image
      new_points = remap_points(acquired_points);

        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;

      for (const auto &pt : new_points) {
        int u = static_cast<int>(pt[0]);
        int v = static_cast<int>(pt[1]);
        // random color
        

        cv::circle(img, cv::Point(u, v), 1, cv::Scalar(b, g, r), -1);

      // 3. Converti di nuovo in ROS Image e pubblica


      }
    }

    sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();
    image_pub_->publish(*out_msg);

  //     // Remap the acquired points to the camera image
  //     new_points = remap_points(acquired_points, points_intensity);
  //     RCLCPP_INFO(this->get_logger(), "Received synchronized messages");

  // if (debug_) {
  //   // 1. Converti sensor_msgs::Image → cv::Mat
  //   cv_bridge::CvImagePtr cv_ptr;
  //   try {
  //     cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  //   } catch (cv_bridge::Exception &e) {
  //     RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  //     return;
  //   }

  //   cv::Mat img = cv_ptr->image;

  //   // 2. Disegna ogni punto di new_points
  //   for (const auto &pt : new_points) {
  //     int u = static_cast<int>(pt[0]);
  //     int v = static_cast<int>(pt[1]);
  //     float distance = pt[2];
  //     if (distance > 10.0f) { distance = 10.0f; }
  //     distance = distance / 10.0f * 255; // Scale to 0-255 range for color
  //     int color = static_cast<int>(255 - distance);
  //     cv::circle(img, cv::Point(u, v), 1, cv::Scalar(0, color, 0), -1);
  //   }

  //   // 3. Converti di nuovo in ROS Image e pubblica
  //   sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();
  //   image_pub_->publish(*out_msg);
  // }

}


// void RoiClusterFusionNode::syncCallback(
//   const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & cluster_sub_msg,
//   const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr & roi_msg,
//   /* *** */
//   const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
// {
//     std::vector<cv::Vec3f> acquired_points;
//     std::vector<cv::Vec4f> new_points;
//     std::vector<float> points_intensity;
//     sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud_sub_msg, "x");
//     sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointcloud_sub_msg, "y");
//     sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointcloud_sub_msg, "z");
//     sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*pointcloud_sub_msg, "intensity");

//     for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
//         if (*iter_x >= 0
//         && *iter_y >= min_y && *iter_y <= max_y
//         && *iter_z >= min_z && *iter_z <= max_z) {
//             acquired_points.emplace_back(*iter_x, *iter_y, *iter_z);
//             points_intensity.emplace_back(*iter_intensity);
//         }
//     }
//     // Remap the acquired points to the camera image
//     new_points = remap_points(acquired_points, points_intensity);
//     RCLCPP_INFO(this->get_logger(), "Received synchronized messages");

// if (debug_) {
//   // 1. Converti sensor_msgs::Image → cv::Mat
//   cv_bridge::CvImagePtr cv_ptr;
//   try {
//     cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
//   } catch (cv_bridge::Exception &e) {
//     RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
//     return;
//   }

//   cv::Mat img = cv_ptr->image;

//   // 2. Disegna ogni punto di new_points
//   for (const auto &pt : new_points) {
//     int u = static_cast<int>(pt[0]);
//     int v = static_cast<int>(pt[1]);
//     float distance = pt[2];
//     if (distance > 10.0f) { distance = 10.0f; }
//     distance = distance / 10.0f * 255; // Scale to 0-255 range for color
//     int color = static_cast<int>(255 - distance);
//     cv::circle(img, cv::Point(u, v), 1, cv::Scalar(0, color, 0), -1);
//   }

//   // 3. Converti di nuovo in ROS Image e pubblica
//   sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();
//   image_pub_->publish(*out_msg);
// }



// }






std::vector<cv::Vec3f> RoiClusterFusionNode::remap_points(const std::vector<cv::Vec3f> &acquired_points){
    std::vector<cv::Vec3f> *remapped_points = new std::vector<cv::Vec3f>();
    std::vector<cv::Point2f> p2D;


    cv::Mat acquired_points_mat(acquired_points);

    cv::projectPoints(acquired_points_mat, rvec_, tvec_, camera_matrix_, dist_coeffs_, p2D);


    for (size_t i = 0; i < p2D.size(); ++i){
        // p2D[i] *= SCALE_FACTOR;
        float distance = cv::norm(acquired_points[i]);
        remapped_points->emplace_back(p2D[i].x, p2D[i].y, distance);
    }

    return *remapped_points;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoiClusterFusionNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}




// std::vector<cv::Vec4f> RoiClusterFusionNode::remap_points(const std::vector<cv::Vec3f> &acquired_points, const std::vector<float> &points_intensity){
//     std::vector<cv::Vec4f> *remapped_points = new std::vector<cv::Vec4f>();
//     std::vector<cv::Point2f> p2D;


//     cv::Mat acquired_points_mat(acquired_points);

//     cv::projectPoints(acquired_points_mat, rvec_, tvec_, camera_matrix_, dist_coeffs_, p2D);


//     for (size_t i = 0; i < p2D.size(); ++i){
//         p2D[i] *= SCALE_FACTOR;
//         float distance = cv::norm(acquired_points[i]);
//         remapped_points->emplace_back(p2D[i].x, p2D[i].y, distance, points_intensity[i]);
//     }

//     return *remapped_points;
// }


// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<RoiClusterFusionNode>(rclcpp::NodeOptions());
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
