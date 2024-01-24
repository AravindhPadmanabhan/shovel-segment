import open3d as o3d
import numpy as np

def icp_open3d(mesh, point_cloud, max_correspondence_distance):
    """
    Perform ICP registration between a triangular mesh and a point cloud.

    Args:
        mesh (o3d.geometry.TriangleMesh): The triangular mesh.
        point_cloud (o3d.geometry.PointCloud): The point cloud.
        max_correspondence_distance (float): Maximum correspondence points-pair distance.

    Returns:
        np.ndarray: The 4x4 transformation matrix.
    """

    # Convert the triangular mesh to a point cloud
    mesh_pc = mesh.sample_points_poisson_disk(number_of_points=1000)

    # Setting up ICP registration
    convergence_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(1e-8, 1e-8, 1000)
    final_registration = o3d.pipelines.registration.registration_icp(
        mesh_pc, 
        point_cloud, 
        max_correspondence_distance, 
        np.identity(4),  # Initial transformation
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        convergence_criteria
    )

    # Extracting the transformation matrix
    final_transform = final_registration.transformation
    print("Registered transformation:\n", final_transform)

    return final_transform