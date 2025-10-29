import numpy as np
import sys

def test_npz(filepath):
    """Test and display contents of NPZ calibration file"""
    print(f"Testing NPZ file: {filepath}\n")
    
    try:
        # Load the file
        data = np.load(filepath)
        
        print("=" * 60)
        print("NPZ File Contents:")
        print("=" * 60)
        
        # List all keys
        print(f"\nAvailable keys: {list(data.keys())}\n")
        
        # Check for expected keys
        expected_keys = ['mtx', 'dist', 'K', 'D', 'width', 'height']
        found_keys = [key for key in expected_keys if key in data]
        missing_keys = [key for key in expected_keys if key not in data]
        
        if found_keys:
            print(f"Found keys: {found_keys}")
        if missing_keys:
            print(f"Missing keys: {missing_keys}")
        
        print("\n" + "=" * 60)
        print("Data Details:")
        print("=" * 60)
        
        # Display each array
        for key in data.keys():
            value = data[key]
            print(f"\n{key}:")
            print(f"   Type: {type(value)}")
            
            if isinstance(value, np.ndarray):
                print(f"   Shape: {value.shape}")
                print(f"   Dtype: {value.dtype}")
                print(f"   Data:\n{value}")
            else:
                print(f"   Value: {value}")
        
        print("\n" + "=" * 60)
        print("Validation:")
        print("=" * 60)
        
        # Check camera matrix
        K = data.get('mtx') or data.get('K')
        if K is not None:
            if K.shape == (3, 3):
                print("Camera matrix (K/mtx) has correct shape (3, 3)")
                print(f"   Focal length fx: {K[0, 0]:.2f}")
                print(f"   Focal length fy: {K[1, 1]:.2f}")
                print(f"   Principal point cx: {K[0, 2]:.2f}")
                print(f"   Principal point cy: {K[1, 2]:.2f}")
            else:
                print(f"Camera matrix has wrong shape: {K.shape} (expected 3x3)")
        else:
            print("Camera matrix (K or mtx) not found")
        
        # Check distortion
        D = data.get('dist') or data.get('D')
        if D is not None:
            if D.ndim == 1 and len(D) >= 5:
                print(f"Distortion coefficients have correct shape ({len(D)},)")
                print(f"   k1={D[0]:.6f}, k2={D[1]:.6f}, p1={D[2]:.6f}, p2={D[3]:.6f}, k3={D[4]:.6f}")
            elif D.ndim == 2 and D.shape[1] >= 5:
                print(f"Distortion coefficients (2D array): {D.shape}")
                D_flat = D.flatten()
                print(f"   k1={D_flat[0]:.6f}, k2={D_flat[1]:.6f}, p1={D_flat[2]:.6f}, p2={D_flat[3]:.6f}, k3={D_flat[4]:.6f}")
            else:
                print(f"Distortion has unexpected shape: {D.shape}")
        else:
            print("Distortion coefficients (D or dist) not found")
        
        # Check dimensions
        width = data.get('width')
        height = data.get('height')
        if width is not None and height is not None:
            print(f"Image dimensions: {int(width)} x {int(height)}")
        else:
            print("Image dimensions not found in file")
        
        print("\n" + "=" * 60)
        print("NPZ file test complete!")
        print("=" * 60)
        
        return True
        
    except FileNotFoundError:
        print(f"ERROR: File not found: {filepath}")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = '/home/hp/Desktop/Autonomous-Drone/ros_ws/bridges/camera_calib.npz'
    
    test_npz(filepath)