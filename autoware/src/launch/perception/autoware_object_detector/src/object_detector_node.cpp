#include "object_detector_node.hpp"

#include <fstream>
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// #include "autoware/object_recognition_utils/object_classification.hpp"
#include <autoware_perception_msgs/msg/object_classification.hpp>


namespace autoware::object_detector
{

ObjectDetector::ObjectDetector(const rclcpp::NodeOptions & node_options) 
    : Node("object_detector", node_options)
{

    modelPath_ = this->declare_parameter<std::string>("model_path", modelPath_);
    labelsPath_ = this->declare_parameter<std::string>("label_path", labelsPath_);
    bool useGPU_ = this->declare_parameter<bool>("use_gpu", true);
    debug_visualizer_ = this->declare_parameter<bool>("debug_visualizer", false);
    confidenceThreshold_ = this->declare_parameter<float>("confidence_threshold", 0.3f);
    CONFIDENCE_THRESHOLD = confidenceThreshold_;
    std::string autowareLabelsPath = this->declare_parameter<std::string>("autoware_label_path", "");

    if (!isFileExists(modelPath_))
    {
        RCLCPP_ERROR(this->get_logger(), "Model file does not exist: %s", modelPath_.c_str());
        rclcpp::shutdown();
        return;
    }

    if (!isFileExists(labelsPath_))
    {
        RCLCPP_ERROR(this->get_logger(), "Labels file does not exist: %s", labelsPath_.c_str());
        rclcpp::shutdown();
        return;
    }


    if (!autowareLabelsPath.empty() && !isFileExists(autowareLabelsPath))
    {
        RCLCPP_ERROR(this->get_logger(), "Autoware labels file does not exist: %s", autowareLabelsPath.c_str());
        rclcpp::shutdown();
        return;
    }




    yolo12Detector_ = std::make_unique<YOLO12Detector>(modelPath_, labelsPath_, useGPU_);

    std::vector<std::string> autowareLabels = getAutowareClassNames(autowareLabelsPath);
    labelMap_ = labelMapping(yolo12Detector_->getClassNames(), autowareLabels);

    // RCLCPP_INFO(this->get_logger(), "Loaded autoware class names:");
    // for (const auto &label : autowareLabels)
    // {
    //     RCLCPP_INFO(this->get_logger(), "- %s", label.c_str());
    // }   

    objects_pub_ = create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>("~/out/objects", rclcpp::QoS(10));
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>("~/in/image", 10, std::bind(&ObjectDetector::onImage, this, std::placeholders::_1));
    image_visualizer_pub_ = create_publisher<sensor_msgs::msg::Image>("~/out/image_visualizer", 10);

    RCLCPP_INFO(this->get_logger(), "ObjectDetector initialized with model: %s, labels: %s", modelPath_.c_str(), labelsPath_.c_str());
}

bool ObjectDetector::isFileExists(const std::string &filePath)
{
    std::ifstream file(filePath);
    return file.good();
}



std::vector<std::string> ObjectDetector::getAutowareClassNames(std::string &path) {
    
    std::vector<std::string> classNames;
    std::ifstream infile(path);

    if (infile) {

        std::string line;
        while (getline(infile, line)) {
            // Remove carriage return if present (for Windows compatibility)
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
                
            }
            // line in lower case
            std::transform(line.begin(), line.end(), line.begin(), ::tolower);
            classNames.emplace_back(line);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to access class name path: %s", path.c_str());
    }

    if (classNames[0] != "unknown") {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Class name 'unknown' should be the first class in the autoware class names file: %s", path.c_str());
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Loaded autoware class names from: %s", path.c_str());
    }
    return classNames;
}

std::unordered_map<int, int> ObjectDetector::labelMapping(const std::vector<std::string>& classNames,
                                      const std::vector<std::string>& autowareClassNames)
{
    std::unordered_map<int, int> labelMap;
    
    for (size_t j = 1; j < autowareClassNames.size(); ++j) {
        const auto& autowareClassName = autowareClassNames[j];
        
        for (size_t i = 0; i < classNames.size(); ++i) {
            const auto& className = classNames[i];
            if (className == autowareClassName) {
                labelMap[i] = j; // Map the index of classNames to autowareClassNames
                break; // Stop searching once a match is found
            }
        }
    }

                            

    return labelMap;
}



void ObjectDetector::onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
    if (!msg)
    {
        RCLCPP_ERROR(this->get_logger(), "Received null image message");
        return;
    }

    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (image.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
        return;
    }


    std::vector<Detection> detections = yolo12Detector_->detect(image, confidenceThreshold_);
    std::vector<Detection> filtered_detections;
    tier4_perception_msgs::msg::DetectedObjectsWithFeature detected_objects;
    
    for (const auto &detection : detections)
    {
        int label = labelMap_[detection.classId];
        if(label == 0) // Skip 'unknown' class
        {
            continue;
        }
        autoware_perception_msgs::msg::ObjectClassification classification_msg;
        tier4_perception_msgs::msg::DetectedObjectWithFeature obj;
        obj.feature.roi.x_offset = detection.box.x;
        obj.feature.roi.y_offset = detection.box.y;
        obj.feature.roi.width = detection.box.width;
        obj.feature.roi.height = detection.box.height;
        obj.object.existence_probability = detection.conf;
        classification_msg.label = label;
        classification_msg.probability = 1.0f;
        obj.object.classification.push_back(classification_msg);
        detected_objects.feature_objects.push_back(obj);
        if (debug_visualizer_)
        {
            filtered_detections.push_back(detection);
        }
    }

    detected_objects.header = msg->header;


    objects_pub_->publish(detected_objects);

    if(debug_visualizer_)
    {
        // Draw bounding boxes on the image
        yolo12Detector_->drawBoundingBox(image, filtered_detections);

        // Convert cv::Mat to sensor_msgs::msg::Image
        sensor_msgs::msg::Image image_msg;
        image_msg.header = msg->header;
        image_msg.height = image.rows;
        image_msg.width = image.cols;
        image_msg.encoding = "bgr8";
        image_msg.is_bigendian = false;
        image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * 3);
        image_msg.data.resize(image.rows * image.cols * 3);
        std::memcpy(image_msg.data.data(), image.data, image.rows * image.cols * 3);

        // Publish the visualized image
        image_visualizer_pub_->publish(image_msg);
    }
}




} // namespace autoware::object_detector



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware::object_detector::ObjectDetector>(
    rclcpp::NodeOptions().use_intra_process_comms(true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}