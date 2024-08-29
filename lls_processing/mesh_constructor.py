import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import TransformListener, LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer

from sensor_msgs.msg import PointCloud2, PointField
from keyboard_msgs.msg import Key

import open3d as o3d
import numpy as np
import copy 


# TODO
    # Try to get the offset from the flange to the LLS
    

class MeshConstructor(Node):
    def __init__(self):
        super().__init__('mesh_constructor')

        pcl_topic = 'scancontrol_pointcloud'
        self.create_subscription(PointCloud2, pcl_topic, self.pcl_callback, 1)
        self.combined_pcl_publisher = self.create_publisher(PointCloud2, 'combined_cloud', 1)
        # self.create_subscription(JointState, 'joint_states', self.update_position, 1)
        self.keyboard_listener  = self.create_subscription(Key, 'keydown', self.key_callback, 1)

        
        # Track coordinate transformations
        self.tf_buffer = Buffer(cache_time=rclpy.time.Time(seconds=3))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_frame_id = 'base_link'

        # Collect scanned pointclouds in format: [(PointCloud2, self.current_pose)]
        # This should be emptied once the points have been sent to transformed_pcls
        self.scanned_data = []
        
        # The PCLs that have been set to a common coordinate system and combined
        self.transformed_pcls: list[o3d.geometry.PointCloud] = [] 

        # A collection of meshes to be compared
        # Each of these is a finished mesh compiled from all PCL data of a measurement interval
        self.alpha = 0.01
        self.meshes: list[o3d.geometry.TriangleMesh] = []

        # TODO FIX VECTOR FLIP IN THE SCANCONTROL DRIVER AND REMOVE THIS LATER
        self.flip_axes = True 


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

    def combine_pointclouds(self) -> None:
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
            
            loaded_array = np.frombuffer(pointcloud.data, dtype=np.float32).reshape(-1, 4)

            # TODO quick fix - remove later
            if self.flip_axes:
                loaded_array[:,0] = - loaded_array[:,0]
                loaded_array[:,2] = - loaded_array[:,2]

                        
            o3d_pcl = o3d.geometry.PointCloud()
            o3d_pcl.points = o3d.utility.Vector3dVector(loaded_array[:,:3])
            o3d_pcl.transform(tf_mat)

            o3d_combined_pcl += o3d_pcl
       
        # Voxel downsampling would be prefered but it is not working for some reason 
        o3d_combined_pcl = o3d_combined_pcl.remove_non_finite_points().random_down_sample(0.1)#.voxel_down_sample(voxel_size=0.1)

        # Add to the stored pcl_list
        self.transformed_pcls.append(o3d_combined_pcl)

        # Publish combined cloud for debugging
        self.combined_pcl_publisher.publish(self.create_pcl_msg(o3d_combined_pcl))
        
        # o3d.visualization.draw_geometries([o3d_combined_pcl], width=800, height=600)

        self.get_logger().info(f"Published combined pointcloud containing {len(o3d_combined_pcl.points)}")
        return 
        
    def construct_mesh(self) -> None:
        # Construct a mesh of the transformed pointcloud 

        self.get_logger().info("Constructing a triangle mesh")
        mesh = o3d.geometry.TriangleMesh()
        mesh.create_from_cloud_alpha_shape(pcd=self.transformed_pcl, alpha=self.alpha)
        self.meshes.append(mesh)
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
            self.combine_pointclouds()

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
        pointcloud.is_dense = True
        pointcloud.height = 1
        pointcloud.width = total_points

        # self.is_bigendian = False
        pointcloud.data = datapoints.flatten().tobytes()

        return pointcloud


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    ===========================================================================================================
    TAKEN FROM THE ROS2 REPO: 
    https://github.com/ros2/geometry2/blob/rolling/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py
    simply importing it led to issues because it is not in the setup file. 
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
    
