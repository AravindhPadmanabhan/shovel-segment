import torch
from tqdm import tqdm
from geotransformer.utils.torch import release_cuda, to_cuda

def infer_transformation(model, test_loader):
    model.eval()
    torch.set_grad_enabled(False)
    total_iterations = len(test_loader)
    pbar = tqdm(enumerate(test_loader), total=total_iterations)
    iterations = 0
    results = []
    
    for iteration, data_dict in pbar:
        iterations = iteration + 1
        data_dict = to_cuda(data_dict)
        torch.cuda.synchronize()
        output_dict = model(data_dict)
        results.append(output_dict['estimated_transform'].cpu().numpy())
        print("ESTIMATED TRANSFORM:\n",output_dict['estimated_transform'].cpu().numpy())
        torch.cuda.synchronize()
        torch.cuda.empty_cache()

    return results[0]
