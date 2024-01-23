import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import torch
import torch.nn as nn
import sys
import pcl
from pcl import PointCloud
import pcl_helper
import open3d as o3d
import numpy as np
from dataset import ShovelDataset
from dataloader import return_data_loader
from tester import infer_transformation
import tf
import tf_conversions
from geometry_msgs.msg import TransformStamped
from registration.msg import TransformationMatrix  # Replace with your package name

experiments_path = os.path.abspath(os.path.join('GeoTransformer', 'experiments', 'geotransformer.modelnet.rpmnet.stage4.gse.k3.max.oacl.stage2.sinkhorn'))
sys.path.insert(0, experiments_path)

from model import create_model
from config import make_cfg

class Registration:
    def __init__(self):
        rospy.init_node('registration_node', anonymous=True)
        self.path_to_mesh = '/home/aravindh/ETH/Semester Project/data/final_mesh.ply'
        self.path_to_weights = '/home/aravindh/ETH/Semester Project/geotransformer/geotransformer_python/weights/180_10_0.85.tar'

        # Initialize shovel mesh
        self.mesh = o3d.io.read_triangle_mesh(self.path_to_mesh)

        # Initialize geotransformer model
        self.cfg = make_cfg()
        self.model = create_model(cfg).cuda()
        state_dict = torch.load(self.path_to_weights, map_location=torch.device('cpu'))
        assert 'model' in state_dict, 'No model can be loaded.'
        self.model.load_state_dict(state_dict['model'], strict=True)

        # Initialize subscriber and publisher
        self.subscriber = rospy.Subscriber('"/livox/lidar_192_168_10_114/segmented_pc"', PointCloud2, self.registration_inference)
        self.publisher = rospy.Publisher('registration/transformation', TransformationMatrix, queue_size=10)
        self.br = tf.TransformBroadcaster()

    def registration_inference(self, data):

        rospy.loginfo('Received point cloud data')

        try:
            # Make open3d point cloud
            pcl_cloud = pcl_helper.ros_to_pcl(data)
            np_points = pcl_helper.pcl_to_np(pcl_cloud)
            o3d_cloud = o3d.geometry.PointCloud()
            o3d_cloud.points = o3d.utility.Vector3dVector(np_points[:, :3])

            # Infer registration
            dataset = ShovelDataset(o3d_cloud, self.mesh.vertices, self.mesh.triangles)
            test_loader = return_data_loader(dataset)
            transformation = infer_transformation(self.model, test_loader)

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
        transform_stamped.header.frame_id = 'SHOVEL_REG'  # Replace with your source frame
        transform_stamped.child_frame_id = 'BASE'  # Replace with your target frame

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