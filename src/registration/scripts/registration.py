#!/home/aravindh/ETH/Semester Project/segmentation/gt/bin/python3

import os
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import open3d as o3d

print("Python interpreter:", sys.executable)

scripts_path = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, scripts_path)

from icp import icp_open3d
import tf
import tf_conversions
from geometry_msgs.msg import TransformStamped
import ros_numpy
from registration.msg import TransformationMatrix

class Registration:
    def __init__(self):
        rospy.init_node('registration', anonymous=True)
        self.path_to_mesh = '/home/aravindh/ETH/Semester Project/data/final_mesh.ply'
        self.path_to_weights = '/home/aravindh/ETH/Semester Project/geotransformer/geotransformer_python/weights/180_10_0.85.tar'
        self.max_correspondence_distance = 10.0

        # Initialize shovel mesh
        self.mesh = o3d.io.read_triangle_mesh(self.path_to_mesh)

        # Initialize subscriber and publisher
        self.subscriber = rospy.Subscriber('"/livox/lidar_192_168_10_114/segmented_pc"', PointCloud2, self.registration_inference)
        self.publisher = rospy.Publisher('registration/transformation', TransformationMatrix, queue_size=10)
        self.br = tf.TransformBroadcaster()

    def registration_inference(self, data):

        rospy.loginfo('Received point cloud data')

        try:
            # Make open3d point cloud
            np_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
            o3d_cloud = o3d.geometry.PointCloud()
            o3d_cloud.points = o3d.utility.Vector3dVector(np_points)

            # Infer registration
            transformation = icp_open3d(self.mesh, o3d_cloud, self.max_correspondence_distance)

            # Publish tf message
            transform = self.tf_message(transformation)
            time_stamp = rospy.Time.now()
            self.br.sendTransformMessage(transform, time_stamp)

            # Publish custom message
            msg = TransformationMatrix()
            msg.source_frame = "SHOVEL_REG"
            msg.registration_frame = "BASE"
            msg.matrix = transformation.flatten().tolist()
            msg.stamp = time_stamp
            self.publisher.publish(msg)

        except Exception as e:
            rospy.logerr('Failed to process point cloud: %s' % str(e))
            self.publisher.publish(String('Registration failed'))

    def tf_message(self, transformation, timestamp):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = timestamp
        transform_stamped.header.frame_id = 'SHOVEL_REG'
        transform_stamped.child_frame_id = 'BASE'

        # Set the translation
        transform_stamped.transform.translation.x = transformation[0, 3]
        transform_stamped.transform.translation.y = transformation[1, 3]
        transform_stamped.transform.translation.z = transformation[2, 3]

        # Set the rotation (quaternion)
        quaternion = tf_conversions.transformations.quaternion_from_matrix(transformation)
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        return transform_stamped

if __name__ == '__main__':
    # Maybe load path files here and send as arguments to Registration()
    registration_node = Registration()
    rospy.spin()