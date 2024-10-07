import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tf2_ros import TransformListener, LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer

from sensor_msgs.msg import PointCloud2, PointField
from keyboard_msgs.msg import Key
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from data_gathering_msgs.srv import RequestPCL

import open3d as o3d
import numpy as np
import copy 
from datetime import datetime
import os 


class MeshConstructor(Node):
    def __init__(self):
        super().__init__('mesh_constructor')

        # TODO parameterize later 

        pcl_topic = 'scancontrol_pointcloud'
        self.create_subscription(PointCloud2, pcl_topic, self.pcl_callback, 10)
        self.combined_pcl_publisher = self.create_publisher(PointCloud2, 'combined_cloud', 1)
        # self.create_subscription(JointState, 'joint_states', self.update_position, 1)
        self.keyboard_listener  = self.create_subscription(Key, 'keydown', self.key_callback, 1)

        # Replaced by service but here for backwards compatibility
        self.combine_trigger_msg = self.create_subscription(Empty, 'combine_pointclouds', self.combine_pointclouds_msg, 1)
        self.combine_trigger     = self.create_service(RequestPCL, 'combine_pointclouds', self.combine_pointclouds_srv, callback_group=MutuallyExclusiveCallbackGroup())

        # Track coordinate transformations
        self.tf_buffer = Buffer(cache_time=rclpy.time.Time(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_frame_id = 'base_link'

        # Collect scanned pointclouds in format: [(PointCloud2, self.current_pose)]
        # This should be emptied once the points have been sent to transformed_pcls
        self.scanned_data = []
        
        # The PCLs that have been set to a common coordinate system and combined
        self.transformed_pcls: list[o3d.geometry.PointCloud] = [] 

        # Create bounding box to crop the point clouds to 
        crop_cloud = o3d.geometry.PointCloud()
        crop_cloud.points = o3d.utility.Vector3dVector(np.array([[-1, -1, -1],
                                                                 [1, 1, 1]]))
        self.crop_volume = crop_cloud.get_axis_aligned_bounding_box()
        
        # A collection of meshes to be compared
        # Each of these is a finished mesh compiled from all PCL data of a measurement interval
        self.alpha = 0.005
        self.meshes: list[o3d.geometry.TriangleMesh] = []
        self.mesh_folder_path = os.path.join(os.getcwd(), 'meshes')
        self.mesh_filename = 'turbine_blade'

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
        self.combine()

    def combine_pointclouds_srv(self, request, response):
        pcl = self.combine()
        response.success = True 
        response.pointcloud = pcl
        return response

    def combine(self) -> None:
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
            
            # TODO check Nick:
                # frombuffer making twice the required data where all even rows are nonsense
            loaded_array = np.frombuffer(pointcloud.data, dtype=np.float32).reshape(-1, 4) 
            if not np.any(loaded_array > 0):
                continue 
            if not np.any(np.isfinite(loaded_array)):
                continue
            
            loaded_array = loaded_array[np.where(loaded_array[:,2] > 5e-3)]

            o3d_pcl = o3d.geometry.PointCloud()
            o3d_pcl.points = o3d.utility.Vector3dVector(loaded_array[:,:3])
            o3d_pcl.transform(tf_mat)

            o3d_combined_pcl += o3d_pcl
       
        self.get_logger().info("Removing non finite points...")
        o3d_combined_pcl = o3d_combined_pcl.remove_non_finite_points()
        o3d_combined_pcl = o3d_combined_pcl.crop(self.crop_volume)

        pcl_path = os.path.join(self.mesh_folder_path, f'pointcloud_{self.mesh_filename}_{str(datetime.now()).split(".")[0]}.ply')
        self.get_logger().info(f"Saving pointcloud to: {pcl_path}")

        o3d.io.write_point_cloud(pcl_path, o3d_combined_pcl)
        self.get_logger().info("Saved pointcloud")

        # Add to the stored pcl_list
        self.transformed_pcls.append(o3d_combined_pcl)

        # Publish combined cloud for debugging
        # downsampled_pcl = o3d_combined_pcl.random_down_sample(0.3)
        pcl_msg = self.create_pcl_msg(o3d_combined_pcl)
        self.combined_pcl_publisher.publish(pcl_msg)
        
        self.get_logger().info(f"Published combined pointcloud containing {len(o3d_combined_pcl.points)} points")
        return pcl_msg
        
    def construct_mesh(self) -> None:
        # Construct a mesh of the transformed pointcloud 
        if len(self.transformed_pcls) < 1:
            self.get_logger().info("There are no pointclouds stored to create a mesh")
            return 
        
        self.get_logger().info("Constructing a triangle mesh")
        for pointcloud in self.transformed_pcls:
            pcl_path = os.path.join(self.mesh_folder_path, f'pointcloud_{self.mesh_filename}_{str(datetime.now()).split(".")[0]}.ply')
            o3d.io.write_point_cloud(pcl_path, pointcloud)
            self.get_logger().info("Saved pointcloud")
            with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
                self.get_logger().info("Estimating normals")
                pointcloud = pointcloud.voxel_down_sample(0.01)
                pointcloud.estimate_normals()

                # self.get_logger().info("Setting normal orientation")
                # pointcloud.orient_normals_consistent_tangent_plane(100)
                # mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pointcloud, depth=9)
                self.get_logger().info("Creating mesh")
                mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pointcloud, alpha=self.alpha)
            self.meshes.append(mesh)

            # self.get_logger().info(f'The storage path exists: {os.path.isdir(self.mesh_folder_path)}')
            path = os.path.join(self.mesh_folder_path, f'mesh_{self.mesh_filename}_{str(datetime.now()).split(".")[0]}.ply')
            write_success = o3d.io.write_triangle_mesh(path, mesh)

        if write_success:
            self.get_logger().info(f'Meshes created and stored in {self.mesh_folder_path}')
        else:
            self.get_logger().info('Failed to write meshes to file')
        return 

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

    mesh_constructor = MeshConstructor()
    executor = MultiThreadedExecutor()

    rclpy.spin(mesh_constructor, executor=executor)
    mesh_constructor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
