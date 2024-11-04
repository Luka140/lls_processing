# lls_processing ROS2 Package

The `lls_processing` package provides a ROS2 node that subscribes to point cloud data, combines it into a unified coordinate frame, and optionally publishes, saves the combined point cloud. 
The package uses Open3D for point cloud manipulation, and compiles pointclouds using TF2 transforms based on the subscribed pointcloud timestamps. 
Mesh creation has not been implemented yet. 

## Package Contents
- **Node**: `MeshConstructor`  
  Combines multiple point clouds into a single coordinate frame and applies transformations for mesh creation.

## Installation

1. Ensure that you have ROS2 installed.
2. Install dependencies, particularly:
   - `open3d` for point cloud manipulation
   - `numpy` for data processing
   - `data_gathering_msgs` 
   - `keyboard_msgs` (optional)
```bash
pip install open3d==0.18.0 numpy==1.24.0
cd src
git clone git@github.com:Luka140/data_gathering_msgs.git
git clone git@github.com:cmower/ros2-keyboard.git
```

3. Clone this repository into your ROS2 workspace and build it:

   ```bash
   colcon build --packages-select lls_processing
   source install/setup.bash
   ```
   
## Parameters

|    Parameter    |  Type  |  Default  |                                                   Description                                                  |
|:---------------:|:------:|:---------:|:--------------------------------------------------------------------------------------------------------------:|
| global_frame_id | string | base_link | Frame ID for the global coordinate frame to transform the combined cloud into.                                 |
| bbox_max        | float  | 0.0       | Maximum bounding box limit for the combined pointcloud. Points outside the box are removed 0.0 means no limit. |
| local_bbox_max  | float  | 0.0       | Same as the previous but applies to individual pointclouds in their own reference frame                        |
| save_pcl        | bool   | true      | If true, saves the combined pointcloud to a `.ply` file                                                        |
| publish_pcl     | bool   | true      | if true, publishes the combined pointcloud on the `combined_cloud` topic                                       |
| listen_keyboard | bool   | true      | If true, listens for keyboard inputs to trigger point cloud processing operations.                             |

## Topics
### Subscribed Topics
- `/scancontrol_pointcloud` (sensor_msgs/PointCloud2): Receives individual point clouds for processing.
- `/keydown` (keyboard_msgs/Key): Listens for keyboard events (optional).
- `/combine_pointclouds` (std_msgs/Empty): Deprecated topic for triggering point cloud combination.

### Published Topics
- `/combined_cloud` (sensor_msgs/PointCloud2): Publishes the combined point cloud after processing if `publish_pcl` is True

## Services
### Provided Services
- `/combine_pointclouds` (data_gathering_msgs/srv/RequestPCL): A service to request the combination and retrieval of processed point clouds.

## Usage
To launch the node, use:
```bash
ros2 launch lls_processing.launch.py
```
To request a pointcloud:
```bash 
ros2 service call /combine_pointclouds data_gathering_msgs/srv/RequestPCL
```


### Keyboard Controls (Optional)
If listen_keyboard is enabled, the following keys can trigger specific actions:
- C: Trigger point cloud combination.
- M: Trigger mesh construction (not yet implemented).
