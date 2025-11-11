#!/usr/bin/env python3

import cv2
import numpy as np
import glob
import os
import torch
from scipy.optimize import least_squares


def load_midas_model(model_type="MiDaS_small"):
    midas = torch.hub.load("intel-isl/MiDaS", model_type)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    midas.to(device)
    midas.eval()
    
    midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
    if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
        transform = midas_transforms.dpt_transform
    else:
        transform = midas_transforms.small_transform
    
    return midas, transform, device


def estimate_midas_depth(img, midas, transform, device):
    input_batch = transform(img).to(device)
    
    with torch.no_grad():
        prediction = midas(input_batch)
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
    
    output = prediction.cpu().numpy()
    return output


def compute_scale_shift(midas_depth, real_depth, min_depth=0.1, max_depth=10.0):
    mask = (real_depth > min_depth) & (real_depth < max_depth) & (midas_depth > 0)
    
    if mask.sum() < 100:
        raise ValueError(f"Not enough valid depth pixels ({mask.sum()}). Need at least 100.")
    
    midas_valid = midas_depth[mask].flatten()
    real_valid = real_depth[mask].flatten()
    
    midas_inv = 1.0 / midas_valid
    
    def residuals(params):
        scale, shift = params
        predicted = scale * midas_inv + shift
        return predicted - real_valid
    
    result = least_squares(residuals, [1.0, 0.0], method='lm')
    scale, shift = result.x
    
    predicted = scale * midas_inv + shift
    rmse = np.sqrt(np.mean((predicted - real_valid) ** 2))
    mae = np.mean(np.abs(predicted - real_valid))
    
    return scale, shift, rmse, mae, mask.sum()


def calibrate_midas_to_real_depth(esp32_images, realsense_depth_paths, midas, transform, device):
    all_scales = []
    all_shifts = []
    
    print(f"\nProcessing {len(esp32_images)} image pairs for MiDaS calibration...")
    
    for idx, (esp32_img_path, rs_depth_path) in enumerate(zip(esp32_images, realsense_depth_paths)):
        if not os.path.exists(rs_depth_path):
            print(f"  Warning: Depth map {rs_depth_path} not found, skipping")
            continue
        
        img = cv2.imread(esp32_img_path)
        if img is None:
            print(f"  Warning: Could not load image {esp32_img_path}, skipping")
            continue
        
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        print(f"  [{idx+1}/{len(esp32_images)}] Processing {os.path.basename(esp32_img_path)}...")
        midas_depth = estimate_midas_depth(img_rgb, midas, transform, device)
        
        real_depth = np.load(rs_depth_path)
        
        if midas_depth.shape != real_depth.shape:
            real_depth = cv2.resize(real_depth, (midas_depth.shape[1], midas_depth.shape[0]))
        
        try:
            scale, shift, rmse, mae, n_pixels = compute_scale_shift(midas_depth, real_depth)
            all_scales.append(scale)
            all_shifts.append(shift)
            print(f"    Scale: {scale:.4f}, Shift: {shift:.4f}, RMSE: {rmse:.4f}m, MAE: {mae:.4f}m, Pixels: {n_pixels}")
        except ValueError as e:
            print(f"    Error: {e}")
    
    if len(all_scales) == 0:
        raise ValueError("No valid calibration pairs found")
    
    median_scale = np.median(all_scales)
    median_shift = np.median(all_shifts)
    mean_scale = np.mean(all_scales)
    mean_shift = np.mean(all_shifts)
    
    return median_scale, median_shift, mean_scale, mean_shift, all_scales, all_shifts


def apply_calibration(midas_depth, scale, shift):
    midas_inv = 1.0 / (midas_depth + 1e-6)
    real_depth = scale * midas_inv + shift
    real_depth[real_depth < 0] = 0
    return real_depth


def save_midas_calibration(output_path, scale, shift, mean_scale, mean_shift):
    np.savez(
        output_path,
        scale=scale,
        shift=shift,
        mean_scale=mean_scale,
        mean_shift=mean_shift
    )
    print(f"\nMiDaS calibration saved to: {output_path}")


def load_midas_calibration(input_path):
    data = np.load(input_path)
    return data['scale'], data['shift']


def main():
    print("=" * 60)
    print("MiDaS Depth Calibration to Real-World Units")
    print("=" * 60)
    
    esp32_images = sorted(glob.glob('calibration_images/esp32_*.png'))
    realsense_depth = sorted(glob.glob('calibration_images/realsense_depth_*.npy'))
    
    if not esp32_images:
        print("Error: No ESP32 images found!")
        print("Expected: esp32_XX.png in current directory")
        return
    
    if not realsense_depth:
        print("Error: No RealSense depth maps found!")
        print("Expected: realsense_depth_XX.npy in current directory")
        return
    
    print(f"Found {len(esp32_images)} ESP32 images")
    print(f"Found {len(realsense_depth)} RealSense depth maps\n")
    
    print("Loading MiDaS model...")
    midas, transform, device = load_midas_model("MiDaS_small")
    print(f"Model loaded on: {device}\n")
    
    scale, shift, mean_scale, mean_shift, all_scales, all_shifts = calibrate_midas_to_real_depth(
        esp32_images, realsense_depth, midas, transform, device
    )
    
    print("\n" + "=" * 60)
    print("Calibration Results")
    print("=" * 60)
    print(f"\nMedian Scale: {scale:.6f}")
    print(f"Median Shift: {shift:.6f}")
    print(f"Mean Scale:   {mean_scale:.6f}")
    print(f"Mean Shift:   {mean_shift:.6f}")
    print(f"\nScale Std Dev: {np.std(all_scales):.6f}")
    print(f"Shift Std Dev: {np.std(all_shifts):.6f}")
    
    save_midas_calibration('src/perception/calibration/midas_calibration.npz', scale, shift, mean_scale, mean_shift)
    
    print("\n" + "=" * 60)
    print("Usage Example")
    print("=" * 60)
    print("\nimport numpy as np")
    print("from calibrate_depth import load_midas_model, estimate_midas_depth, apply_calibration, load_midas_calibration")
    print("\nmidas, transform, device = load_midas_model('MiDaS_small')")
    print("scale, shift = load_midas_calibration('midas_calibration.npz')")
    print("\nmidas_depth = estimate_midas_depth(img_rgb, midas, transform, device)")
    print("real_depth_meters = apply_calibration(midas_depth, scale, shift)")
    
    print("\n" + "=" * 60)
    print("Calibration complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
