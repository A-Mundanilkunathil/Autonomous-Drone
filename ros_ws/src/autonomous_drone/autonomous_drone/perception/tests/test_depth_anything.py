from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import torch
from PIL import Image
import requests
import time
import numpy as np
import cv2

# -----------------------------
# Model checkpoints to compare
# -----------------------------
model_sets = {
    # NEW: regular (relative) Depth Anything V2
    "Depth Anything V2 (relative)": [
        "depth-anything/Depth-Anything-V2-Small-hf",
        "depth-anything/Depth-Anything-V2-Base-hf",
        "depth-anything/Depth-Anything-V2-Large-hf",
    ],
    # Your original metric indoor fine-tunes
    "Depth Anything V2 (metric indoor)": [
        "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
        "depth-anything/Depth-Anything-V2-Metric-Indoor-Base-hf",
        "depth-anything/Depth-Anything-V2-Metric-Indoor-Large-hf",
    ],
    # MiDaS family via torch.hub
    "MiDaS": [
        "MiDaS_small",  # lightweight and fast
        "DPT_Hybrid",
        "DPT_Large",
    ],
}

# -----------------------------
# Load an example image
# -----------------------------
url = "http://images.cocodataset.org/val2017/000000039769.jpg"
image = Image.open(requests.get(url, stream=True).raw).convert("RGB")
orig_h, orig_w = image.size[1], image.size[0]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
use_amp = (device.type == "cuda")

def resize_to_image(pred_tensor, size_hw):
    """
    pred_tensor: torch.Tensor with shape:
        - (B, H, W) or (B, 1, H, W) or (H, W)
    size_hw: tuple (H, W) target
    returns: (H, W) numpy float32
    """
    if pred_tensor.ndim == 2:
        pred_tensor = pred_tensor.unsqueeze(0).unsqueeze(0)  # (1,1,H,W)
    elif pred_tensor.ndim == 3:
        # assume (B,H,W) -> add channel
        pred_tensor = pred_tensor.unsqueeze(1)
    # Now (B,1,H,W)
    pred_resized = torch.nn.functional.interpolate(
        pred_tensor, size=size_hw, mode="bicubic", align_corners=False
    )
    return pred_resized.squeeze().detach().float().cpu().numpy()

for model_family, model_ids in model_sets.items():
    print(f"\n=== Testing {model_family} Models ===")
    for model_id in model_ids:
        print(f"Model: {model_id}")

        start_time = None
        end_time = None
        predicted_depth = None

        if model_family.startswith("Depth Anything V2"):
            # Hugging Face transformers path
            try:
                # Faster pre/postprocessor
                processor = AutoImageProcessor.from_pretrained(model_id, use_fast=True)
                model = AutoModelForDepthEstimation.from_pretrained(model_id).to(device).eval()

                # Prepare inputs
                inputs = processor(images=image, return_tensors="pt")
                inputs = {k: v.to(device, non_blocking=True) for k, v in inputs.items()}

                # Warmup (optional, helps stabilize timings esp. on GPU)
                with torch.no_grad():
                    _ = model(**inputs)

                # Timed inference (AMP on CUDA)
                start_time = time.time()
                with torch.no_grad():
                    if use_amp:
                        with torch.cuda.amp.autocast(dtype=torch.float16):
                            outputs = model(**inputs)
                    else:
                        outputs = model(**inputs)
                end_time = time.time()

                predicted_depth = outputs.predicted_depth  # (B, H, W)
            except Exception as e:
                print(f"Failed loading DA-V2 {model_id}: {e}")
                continue

        else:
            # MiDaS via torch.hub
            try:
                midas = torch.hub.load("intel-isl/MiDaS", model_id)
                midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

                # Choose correct transform
                if "small" in model_id.lower():
                    transform = midas_transforms.small_transform
                else:
                    transform = midas_transforms.dpt_transform

                midas.to(device).eval()

                # Convert PIL -> numpy RGB for MiDaS transforms
                img_rgb = np.array(image)
                input_tensor = transform(img_rgb)
                # Ensure shape (B, C, H, W) tensor on device
                if torch.is_tensor(input_tensor):
                    if input_tensor.ndim == 3:
                        input_tensor = input_tensor.unsqueeze(0)
                    input_tensor = input_tensor.to(device, non_blocking=True)
                else:
                    input_tensor = torch.from_numpy(np.asarray(input_tensor)).permute(2,0,1).unsqueeze(0).float().to(device)

                # Warmup
                with torch.no_grad():
                    _ = midas(input_tensor)

                # Timed inference
                start_time = time.time()
                with torch.no_grad():
                    if use_amp:
                        with torch.cuda.amp.autocast(dtype=torch.float16):
                            depth_pred = midas(input_tensor)  # often (B, 1, H, W)
                    else:
                        depth_pred = midas(input_tensor)
                end_time = time.time()

                predicted_depth = depth_pred
            except Exception as e:
                print(f"Failed loading MiDaS {model_id}: {e}")
                continue

        # -----------------------------
        # Resize to original image size
        # -----------------------------
        if isinstance(predicted_depth, torch.Tensor):
            depth_map = resize_to_image(predicted_depth, (orig_h, orig_w))
        else:
            # Fallback: force through torch then resize
            pred_t = torch.from_numpy(np.array(predicted_depth)).float()
            depth_map = resize_to_image(pred_t, (orig_h, orig_w))

        # -----------------------------
        # Report
        # -----------------------------
        print(f"Depth map shape: {depth_map.shape}")
        print(f"Inference time: {(end_time - start_time)*1000:.2f} ms\n")
