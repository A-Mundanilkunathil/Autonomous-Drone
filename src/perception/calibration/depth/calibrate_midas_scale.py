#!/usr/bin/env python3

import os
import csv
import cv2
import torch
import numpy as np
from scipy.optimize import curve_fit

SAVE_DIR = "src/perception/calibration/esp32_calib_wall"
MANIFEST = os.path.join(SAVE_DIR, "manifest.csv")
OUT_NPZ = os.path.join(SAVE_DIR, "esp32_midas_calibration.npz")

def load_midas(model_type="MiDaS_small"):
    midas = torch.hub.load("intel-isl/MiDaS", model_type)
    midas.eval()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    midas.to(device)
    transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
    transform = transforms.small_transform if "small" in model_type.lower() else transforms.dpt_transform
    return midas, transform, device

def midas_depth(img_bgr, midas, transform, device):
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    inp = transform(rgb).to(device)
    with torch.no_grad():
        pred = midas(inp)
        pred = torch.nn.functional.interpolate(
            pred.unsqueeze(1),
            size=img_bgr.shape[:2],
            mode="bicubic",
            align_corners=False
        ).squeeze().cpu().numpy()
    return pred

def central_inv_mean(depth):
    h, w = depth.shape
    y0, y1 = h//3, 2*h//3
    x0, x1 = w//3, 2*w//3
    patch = depth[y0:y1, x0:x1]
    inv = 1.0 / (patch + 1e-6)
    flat = np.sort(inv.flatten())
    n = len(flat)
    a, b = int(0.1*n), int(0.9*n)
    return float(np.mean(flat[a:b]))

def main():
    if not os.path.exists(MANIFEST):
        raise FileNotFoundError(f"{MANIFEST} not found. Run capture_known_distance_esp32.py first.")

    fns, dists = [], []
    with open(MANIFEST, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            fns.append(os.path.join(SAVE_DIR, row["filename"]))
            dists.append(float(row["distance_m"]))
    dists = np.array(dists, dtype=np.float32)

    if len(fns) < 4:
        raise RuntimeError("Need at least 4 images at different distances for calibration.")

    print(f"Found {len(fns)} images for calibration")
    print("Loading MiDaS model...")
    midas, tfm, device = load_midas("MiDaS_small")
    print(f"Model loaded on: {device}\n")

    inv_means = []
    for i, p in enumerate(fns):
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print(f"Warning: Could not load {p}")
            continue
        d = midas_depth(img, midas, tfm, device)
        inv_mean = central_inv_mean(d)
        inv_means.append(inv_mean)
        print(f"[{i+1}/{len(fns)}] {os.path.basename(p)}: inv_mean={inv_mean:.6f}, distance={dists[i]:.2f}m")
    
    inv_means = np.array(inv_means, dtype=np.float32)

    def model(inv, a, b):
        return a * inv + b
    
    popt, pcov = curve_fit(model, inv_means, dists, maxfev=10000)
    a, b = popt
    
    pred = model(inv_means, a, b)
    mae = np.mean(np.abs(pred - dists))
    rmse = np.sqrt(np.mean((pred - dists)**2))
    
    print(f"\nCalibration Results:")
    print(f"  Scale (a): {a:.6f}")
    print(f"  Shift (b): {b:.6f}")
    print(f"  MAE: {mae:.3f}m")
    print(f"  RMSE: {rmse:.3f}m")

    np.savez(OUT_NPZ, scale=a, shift=b, mae=mae, rmse=rmse, n=len(inv_means))
    print(f"\nCalibration saved to: {OUT_NPZ}")
    print("\nNext step: Use use_calibrated_depth.py for live depth estimation")

if __name__ == "__main__":
    main()
