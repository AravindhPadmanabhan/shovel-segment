import torch
from torch.utils.data import Dataset
import open3d as o3d
import numpy as np
import os


class ShovelDataset(torch.utils.data.Dataset):

    def __init__(
        self,
        data: str,
        mesh_vertices,
        mesh_faces,
        num_points: int = 717,
        return_normals: bool = False
    ):
        super(ShovelDataset, self).__init__()

        self.data = data
        self.num_points = num_points
        self.pc_list = [data]
        self.return_normals = return_normals

        self.mesh = o3d.geometry.TriangleMesh()
        self.mesh.vertices = o3d.utility.Vector3dVector(mesh_vertices)
        self.mesh.triangles = o3d.utility.Vector3iVector(mesh_faces)
        self.mesh.compute_vertex_normals()

    def __getitem__(self, index):

        src_pcd = self.pc_list[index]
        src_points = np.asarray(src_pcd.points)
        # src_points = self.normalize_points(src_points)
        raw_points = src_points
        if self.return_normals:
            src_pcd.estimate_normals()
            src_normals = np.asarray(src_pcd.normals)

        ref_pcd = self.mesh.sample_points_uniformly(src_points.shape[0])
        ref_points = np.asarray(ref_pcd.points)
        # ref_points = self.normalize_points(ref_points)
        if self.return_normals:
            ref_pcd.estimate_normals()
            ref_normals = np.asarray(ref_pcd.normals)

        new_data_dict = {
            'raw_points': raw_points.astype(np.float32),
            'ref_points': ref_points.astype(np.float32),
            'src_points': src_points.astype(np.float32),
            'ref_feats': np.ones_like(ref_points[:, :1]).astype(np.float32),
            'src_feats': np.ones_like(src_points[:, :1]).astype(np.float32),
            'transform': np.eye(4).astype(np.float32),
            'label': int(0),
            'index': int(index),
        }

        if self.return_normals:
            new_data_dict['ref_normals'] = ref_normals
            new_data_dict['src_normals'] = src_normals

        return new_data_dict

    def __len__(self):
        return len(self.pc_list)
    
    def normalize_points(self, points):
        r"""Normalize point cloud to a unit sphere at origin."""
        # points = points - points.mean(axis=0)
        points = points / np.max(np.linalg.norm(points - points.mean(axis=0), axis=1))
        
        return points