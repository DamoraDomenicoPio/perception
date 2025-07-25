#ifndef AUTOWARE_OBJECT_DETECTOR_HPP
#define AUTOWARE_OBJECT_DETECTOR_HPP

#include "det/YOLO12.hpp" // Uncomment for YOLOv12
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include <unordered_map>

/**
 * @file object_detector_node.hpp
 * @brief Defines the ObjectDetector class for object detection in Autoware.
 */

namespace autoware::object_detector
{

/**
 * @class ObjectDetector
 * @brief ROS 2 node for performing object detection using YOLO12 model.
 *
 * This class subscribes to compressed image topics, performs object detection using a YOLO-based detector,
 * and publishes detected objects with features. It also supports optional debug visualization.
 */
class ObjectDetector : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for ObjectDetector.
     * @param node_options Options to configure the ROS 2 node.
     */
    explicit ObjectDetector(const rclcpp::NodeOptions & node_options);

private:
    std::string modelPath_="";  ///< Path to the YOLO model file.
    std::string labelsPath_=""; ///< Path to the labels file. These labels are the ones that were used to train the model.
    bool debug_visualizer_ = false; ///< Flag to enable or disable debug visualization.
    std::unordered_map<int, int> labelMap_; ///< Mapping between model and Autoware label indices.
                                            //// This mapping is necessary for 2 reasons:
                                            //// 1. The indexes of YOLO12 classes (defined in the coco.names file), and those needed by autoware (defined in the autoware.names file) may be different.
                                            //// 2. It is possible to use a smaller number of classes (since autoware is only interested in some of the YOLO12 classes). The remaining YOLO12 classes will be mapped to "unknown".
    float confidenceThreshold_; ///< Confidence threshold for detections.
    std::unique_ptr<YOLO12Detector> yolo12Detector_; ///< Pointer to the YOLO detector instance.
    rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_; ///< Publisher for detected objects.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_visualizer_pub_; ///< Publisher for visualization images.
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_; ///< Subscriber for compressed image input.
    

    /**
     * @brief Checks if a file exists at the given path.
     * @param filePath Path to the file.
     * @return True if the file exists, false otherwise.
     */
    bool isFileExists(const std::string &filePath);

    /**
     * @brief Callback function for incoming compressed image messages.
     * @param msg Shared pointer to the received compressed image message.
     */
    void onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);

    /**
     * @brief Loads Autoware class names from a file.
     * @attention The first class name in the file must be "unknown".
     * @param path Path to the class names file.
     * @return Vector of class name strings.
     */
    std::vector<std::string> getAutowareClassNames(std::string &path);

    /**
     * @brief Maps model class indices to Autoware class indices.
     * @param classNames Vector of YOLO12 class names.
     * @param autowareClassNames Vector of Autoware class names.
     * @return Unordered map from model class index to Autoware class index.
     */
    std::unordered_map<int, int> labelMapping(const std::vector<std::string>& classNames, const std::vector<std::string>& autowareClassNames);
};

}

#endif // AUTOWARE_OBJECT_DETECTOR_HPP

                                                                                 