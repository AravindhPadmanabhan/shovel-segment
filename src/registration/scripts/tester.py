#!/home/aravindh/ETH/Semester Project/segmentation/gt/bin/python3

# import os
# import sys
import torch as tch
from tqdm import tqdm

# # utils_path = os.path.abspath(os.path.join('GeoTransformer', 'geotransformer', 'utils'))
# utils_path = os.path.abspath(os.path.join('src', 'GeoTransformer', 'geotransformer', 'utils'))
# sys.path.insert(0, utils_path)
from geotransformer.utils.torch import release_cuda, to_cuda

def infer_transformation(model, test_loader):
    model.eval()
    tch.set_grad_enabled(False)
    total_iterations = len(test_loader)
    pbar = tqdm(enumerate(test_loader), total=total_iterations)
    iterations = 0
    results = []
    
    for iteration, data_dict in pbar:
        iterations = iteration + 1
        data_dict = to_cuda(data_dict)
        tch.cuda.synchronize()
        output_dict = model(data_dict)
        results.append(output_dict['estimated_transform'].cpu().numpy())
        print("ESTIMATED TRANSFORM:\n",output_dict['estimated_transform'].cpu().numpy())
        tch.cuda.synchronize()
        tch.cuda.empty_cache()

    return results[0]
