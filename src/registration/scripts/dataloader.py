#!/home/aravindh/ETH/Semester Project/segmentation/gt/bin/python3

# import os
# import sys
from functools import partial
from torch.utils.data import DataLoader
import numpy as np

# utils_path = os.path.abspath(os.path.join('GeoTransformer', 'geotransformer', 'utils'))
# sys.path.insert(0, utils_path)

from geotransformer.utils.data import registration_collate_fn_stack_mode
from geotransformer.utils.torch import reset_seed_worker_init_fn

def return_data_loader(dataset):
    neighbor_limits = np.array([22, 31, 39])
    
    data_loader = DataLoader(
        dataset,
        batch_size=1,
        num_workers=16,
        shuffle=False,
        sampler=None,
        collate_fn=partial(
            registration_collate_fn_stack_mode,
            num_stages=3,
            voxel_size=0.05,
            search_radius=0.125,
            neighbor_limits=neighbor_limits,
            precompute_data=True,
        ),
        worker_init_fn=reset_seed_worker_init_fn,
        pin_memory=False,
        drop_last=False,
    )

    return data_loader
