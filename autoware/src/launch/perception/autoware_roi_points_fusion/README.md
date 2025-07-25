# Roi Points Fusion

## Purpose

This package is responsible for fusing the ROIs detected by the [object detector](../autoware_object_detector/README.md) and the LiDAR data.  
Fusing consists of simply two steps:
1. project the lidar points onto the camera
2. a point is associated with a Roi if it is inside it

An additional node has also been developed for debugging, which displays ROIs with the lidar points associated with them.
## Inputs / Outputs
### Input

| Name       | Type                | Description     |
| ---------- | ------------------- | --------------- |
| `in/lidar` | `sensor_msgs/PointCloud2` | Lidar points |
| `in/rois`  | `tier4_perception_msgs/DetectedObjectsWithFeature` | Rois provided as output from the object detector |
| `in/image` | `sensor_msgs/CompressedImage` | The input image |

### Output

| Name             | Type                                               | Description                                                                                           |
| ---------------- | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `out/rois`    | `tier4_perception_msgs/DetectedObjectsWithFeature` | The detected objects with the associated lidar points                                          |
| `out/visualization`      | `sensor_msgs/Image`                                | The image with bounding boxes with associated lidar points for visualization                                                    |

## Parameters

The following parameters are defined in `config/roi_points_fusion_node.param.yaml`:
- `lidar`:
    - `min_y`, `max_y`, `min_z`, `max_z`: minimum and maximum values on the y and z axis of the lidar points.
- `camera`: 
    - `IMG_WIDTH` and `IMG_HEIGHT`: Height and width of images.
- `calibration`:
    - `R`, `tvec`, `camera_matrix`, `dist_coeffs`: Intrinsic and extrinsic matrices obtained from camera-LiDAR calibration ([see here for more information](../../../../../../calbration/ros2_camera_lidar_fusion/README.md)).
---

<!-- TODO: cambia link -->

## How to use it

If you have never done it, remember to always do the colcon build and source first
```bash
colcon build --packages-select autoware_roi_points_fusion
source install/setup.bash
```

Launch the node with this command:
```bash
ros2 launch autoware_roi_points_fusion roi_points_fusion.launch.xml
```

You can also start the debug node in two different ways:
- edit the launch file in `config/roi_points_fusion_node.param.yaml` by setting the `debug` argument (`<arg name="debug" default="true"/>`) to true, and then launch the node as before.
- specify it explicitly on the command line without modifying the launch file:
    ```bash
    ros2 launch autoware_roi_points_fusion roi_points_fusion.launch.xml debug:=true
    ```