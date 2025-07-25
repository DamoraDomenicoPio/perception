# Object detector

## Purpose

This package detects target objects, such as cars, trucks, bicycles and pedestrians using [YOLO12](https://arxiv.org/abs/2502.12524)

## Inputs / Outputs
### Input

| Name       | Type                | Description     |
| ---------- | ------------------- | --------------- |
| `in/image` | `sensor_msgs/CompressedImage` | The input image |

### Output

| Name             | Type                                               | Description                                                                                           |
| ---------------- | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `out/objects`    | `tier4_perception_msgs/DetectedObjectsWithFeature` | The detected objects with 2D bounding boxes                                          |
| `out/image_visualizer`      | `sensor_msgs/Image`                                | The image with 2D bounding boxes for visualization                                                    |

## Parameters

The following parameters are defined in `config/yolo12.param.yaml`:
- `confidence_threshold`: Minimum confidence threshold to consider a prediction valid. Detections with lower confidence will be discarded.
- `use_gpu`: Enables the use of GPU for inference, if available. If set to false, the model will run on CPU.
- `debug_visualizer`: Enables visualization of results during execution, useful for debugging. Shows images with detected bounding boxes.

---

## How to use it

To use the object detector, you need to take some preliminary steps.

### Onnxruntime installation

To install onnxruntime, go to the `autoware_object_detector` directory and run `install_onnxruntime.sh` specifying the version you want to download.
See [here](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html) to check all cuda version compatibility.

The command to execute:
```bash
./install_onnxruntime.sh x y
```
where:
- `x` is the version of onnxruntime you want to install
- with `y` you can specify if you want to install the version with/without gpu. 0 without gpu, 1 with gpu

Let's assume a version of cuda 12.6 and cuDNN 9.3.0.
In this case you need to run these commands:
```bash
cd autoware/src/universe/autoware_universe/perception/autoware_object_detector/
./install_onnxruntime.sh 1.20.1 1
```

After that, open the `CMakeLists.txt` and specify in `ONNXRUNTIME_VERSION` the downloaded version.
Following the example above, in the `CMakeLists.txt` you will see the line:
```text
set(ONNXRUNTIME_VERSION "1.20.1")
```

> **Warning**
>
> Here it is assumed that onnxruntime is installed on a `linux` platform, with an `x64` architecture and downloading the version with `gpu`.
> In fact in the `CMakeLists.txt` you will find this line:
> ```text
> set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}")
> ```
> In case you install onnxruntime on a different platform, or on a different architecture, then specify the correct values.
> If the gpu version is not installed, then remove it in the name

### Download the onnx model

Download or convert the model version to onnx.
You can download the onnx of different YOLO versions from [here](https://mega.nz/folder/TvgXVRQJ#6M0IZdMOvKlKY9-dx7Uu7Q)

Place the model in the `~/autoware_data/object_detector/models` folder, and in `~/autoware_data/object_detector/labels` both the labels used to train the model (for example, the <b>YOLOv12</b> model used the labels specified in `coco.names` which you can find [here](https://github.com/Geekgineer/YOLOs-CPP/tree/main/models)) and the subset of labels you want to use in autoware (specifying them, for example, in a file called `autoware.names`).

> **Note**
>
> If you want to use a different path, then make sure to also change the `data_path` variable in the launch file.

The format of `autoware.names` must follow two simple rules:
1. the first label must be `UNKNOWN`
2. the other labels must be specified in subsequent lines

An example of an `autoware.names` file (which, by the way, are the labels used by autoware):
```text
UNKNOWN
CAR
TRUCK
BUS
BICYCLE
MOTORBIKE
PEDESTRIAN
ANIMAL
```

## Command to launch the object detector node

If you have never done it, remember to always do the colcon build and source first
```bash
colcon build --packages-select autoware_object_detector
source install/setup.bash
```

Launch the node with this command:
```bash
ros2 launch autoware_object_detector object_detector.launch.xml 
```

