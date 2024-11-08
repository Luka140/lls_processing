import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tf2_ros import TransformListener, LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from data_gathering_msgs.srv import RequestPCL

import open3d as o3d
import numpy as np
import copy 
from datetime import datetime
import os 


class PCLConstructor(Node):
    def __init__(self):
        super().__init__('pcl_constructor')

        # Declare parameters
        self.declare_parameter('global_frame_id', 'base_link')
        self.declare_parameter('alpha', 0.005)
        self.declare_parameter('bbox_max', 0.0)
        self.declare_parameter('local_bbox_max', 0.0)
        self.declare_parameter('save_pcl', True)
        self.declare_parameter('publish_pcl', True)
        self.declare_parameter('listen_keyboard', True)

        # Get parameters
        self.global_frame_id    = self.get_parameter('global_frame_id').get_parameter_value().string_value
        self.alpha              = self.get_parameter('alpha').get_parameter_value().double_value
        self.bbox_max           = self.get_parameter('bbox_max').get_parameter_value().double_value
        self.local_bbox_max     = self.get_parameter('local_bbox_max').get_parameter_value().double_value
        self.save_pcl           = self.get_parameter('save_pcl').get_parameter_value().bool_value
        self.publish_pcl        = self.get_parameter('publish_pcl').get_parameter_value().bool_value
        self.listen_keyboard    = self.get_parameter('listen_keyboard').get_parameter_value().bool_value

        # PCL topic and subscribers
        self.create_subscription(PointCloud2, 'scancontrol_pointcloud', self.pcl_callback, 20)
        self.combined_pcl_publisher = self.create_publisher(PointCloud2, 'combined_cloud', 1)
        if self.listen_keyboard:
            from keyboard_msgs.msg import Key
            self.keyboard_listener = self.create_subscription(Key, 'keydown', self.key_callback, 1)

        # Topic replaced by service but here for backwards compatibility
        self.combine_trigger_msg = self.create_subscription(Empty, 'combine_pointclouds', self.combine_pointclouds_msg, 1)
        self.combine_trigger = self.create_service(RequestPCL, '~/combine_pointclouds', self.combine_pointclouds_srv, callback_group=MutuallyExclusiveCallbackGroup())

        # TF buffer for transformations
        self.tf_buffer = Buffer(cache_time=rclpy.time.Time(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize scanned data and transformed point clouds
        self.scanned_data = []
        self.transformed_pcls: list[o3d.geometry.PointCloud] = []

        # Create a bounding box based on the bbox_max parameter
        if self.bbox_max > 0.0:
            crop_cloud = o3d.geometry.PointCloud()
            crop_cloud.points = o3d.utility.Vector3dVector(np.array([[-self.bbox_max, -self.bbox_max, -self.bbox_max],
                                                                    [self.bbox_max, self.bbox_max, self.bbox_max]]))
            self.crop_volume = crop_cloud.get_axis_aligned_bounding_box()

        if self.local_bbox_max > 0.0:
            local_crop_cloud = o3d.geometry.PointCloud()
            local_crop_cloud.points = o3d.utility.Vector3dVector(np.array([[-self.local_bbox_max, -self.local_bbox_max, -self.local_bbox_max],
                                                                    [self.local_bbox_max, self.local_bbox_max, self.local_bbox_max]]))
            self.local_crop_volume = local_crop_cloud.get_axis_aligned_bounding_box()

    
    def pcl_callback(self, msg):
        
        # If the PCL message is empty 
        if len(msg.data) < 1:
            return

        # Lookup the transformation for the time the pointcloud was created
        time = msg.header.stamp
        try:
            tf_trans = self.tf_buffer.lookup_transform(self.global_frame_id, msg.header.frame_id, time, timeout=rclpy.duration.Duration(seconds=3))
        except (LookupException, ExtrapolationException):
            self.get_logger().info(f"Could not lookup transform for time: {time}")
            return

        self.scanned_data.append((msg, tf_trans))

    def combine_pointclouds_msg(self, _):
        # Set publish to true regardless of setting in launch file
        self.combine(publish_override=True)

    def combine_pointclouds_srv(self, request, response):
        pcl = self.combine()
        response.success = True 
        response.pointcloud = pcl
        return response

    def combine(self, publish_override=False) -> None:
        # Take all points in scanned data and combine them into a single coordinate frame

        # Make a new list to prevent it being extended while in the loop
        current_data = copy.deepcopy(self.scanned_data)
        
        self.get_logger().info(f"Combining {len(current_data)} pointclouds....")

        # Reset the scanned_data list 
        self.scanned_data = []

        o3d_combined_pcl = o3d.geometry.PointCloud()
        for pointcloud, tf_transform in current_data:
        
            # Create a transformation matrix from the transformation message obtained from tf_trans
            tf_mat = self.tf_transform_to_matrix(tf_transform)
            
            loaded_array = np.frombuffer(pointcloud.data, dtype=np.float32).reshape(-1, len(pointcloud.fields)) 
            if not np.any(loaded_array > 0):
                continue 
            if not np.any(np.isfinite(loaded_array)):
                continue
            
            # The scancontrol driver creates pointclouds that are 32 bytes per point, only the first 12 of which are used for XYZ. 
            # It also has an intensity field which currently doesn't have the real data. 
            # The empty data creates nonsense points on every second row from the last 16 empty bytes being interpreted as a point.
            # Bytes 12-16 are currently interpreted as intensity (it isn't) in the fourth column but this isnt used so no problem. 
            if pointcloud.header.frame_id == 'scancontrol':
                loaded_array = loaded_array[0::2, :] 

            # loaded_array = loaded_array[np.where(loaded_array[:,2] > 5e-3)] 

            o3d_pcl = o3d.geometry.PointCloud()
            o3d_pcl.points = o3d.utility.Vector3dVector(loaded_array[:,:3])
            if self.local_bbox_max > 0.0:
                o3d_pcl = o3d_pcl.crop(self.local_crop_volume)
            o3d_pcl.transform(tf_mat)

            o3d_combined_pcl += o3d_pcl
       
        self.get_logger().info("Removing non finite points...")
        o3d_combined_pcl = o3d_combined_pcl.remove_non_finite_points() 
        if self.bbox_max > 0.0:
            o3d_combined_pcl = o3d_combined_pcl.crop(self.crop_volume)

        if self.save_pcl:
            pcl_path = os.path.join(self.mesh_folder_path, f'pointcloud_{self.mesh_filename}_{str(datetime.now()).split(".")[0]}.ply')
            self.get_logger().info(f"Saving pointcloud to: {pcl_path}")

            o3d.io.write_point_cloud(pcl_path, o3d_combined_pcl)
            self.get_logger().info("Saved pointcloud")

        # Add to the stored pcl_list
        self.transformed_pcls.append(o3d_combined_pcl)

        # Publish combined cloud for debugging
        # downsampled_pcl = o3d_combined_pcl.random_down_sample(0.3)
        pcl_msg = self.create_pcl_msg(o3d_combined_pcl)

        if self.publish_pcl or publish_override:
            self.combined_pcl_publisher.publish(pcl_msg)
        
        self.get_logger().info(f"Published combined pointcloud containing {len(o3d_combined_pcl.points)} points")
        return pcl_msg
        
 
    def tf_transform_to_matrix(self, tf_trans) -> np.ndarray:
        transform = np.eye(4)
        rot = tf_trans.transform.rotation
        trans = tf_trans.transform.translation

        # If this is altered - be careful of the order of the quaternion (w,x,y,z) vs (x,y,z,w)
        # Some packages use one, other use the other and they often don't specify which is used.  
        transform[:3,:3] = _get_mat_from_quat(np.array([rot.w, rot.x, rot.y, rot.z]))
        transform[:3,3] = [trans.x, trans.y, trans.z]
        return transform
    
    def key_callback(self, Key):
        if Key.code == Key.KEY_C:
            self.combine_pointclouds(Empty())
        if Key.code == Key.KEY_M:
            self.construct_mesh()

    def create_pcl_msg(self, o3d_pcl):
        datapoints = np.asarray(o3d_pcl.points, dtype=np.float32)

        pointcloud = PointCloud2()
        pointcloud.header.frame_id = self.global_frame_id

        dims = ['x', 'y', 'z']

        bytes_per_point = 4
        fields = [PointField(name=direction, offset=i * bytes_per_point, datatype=PointField.FLOAT32, count=1) for i, direction in enumerate(dims)]
        pointcloud.fields = fields
        pointcloud.point_step = len(fields) * bytes_per_point
        total_points = datapoints.shape[0]
        pointcloud.is_dense = False
        pointcloud.height = 1
        pointcloud.width = total_points

        pointcloud.data = datapoints.flatten().tobytes()
        return pointcloud


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    ===========================================================================================================
    TAKEN FROM THE ROS2 REPO: 
    https://github.com/ros2/geometry2/blob/rolling/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py
    simply importing it led to issues because it is not in the setup file in humble. 
    ===========================================================================================================

    Convert a quaternion to a rotation matrix.

    This method is based on quat2mat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101 ,
    since that library is not available via rosdep.

    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: The rotation matrix
    """
    Nq = np.sum(np.square(quaternion))
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [[1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
        [xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
        [xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])


def main(args=None):
    rclpy.init(args=args)

    mesh_constructor = PCLConstructor()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(mesh_constructor, executor=executor)
    except KeyboardInterrupt:
        pass 
    mesh_constructor.destroy_node()

    # Avoid stack trace 
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass 


if __name__ == '__main__':
    main()
    
