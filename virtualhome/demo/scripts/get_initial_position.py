import argparse
import os

def parse_args():
    parser = argparse.ArgumentParser(description="Print initial position from pose file.")
    parser.add_argument('--pose_path', type=str, help='Path to the pose file (e.g., pd_xxx.txt)')
    return parser.parse_args()

def print_initial_position(pose_path):
    if not os.path.isfile(pose_path):
        print(f"Pose file not found: {pose_path}")
        return
    with open(pose_path, "r") as f:
        lines = f.readlines()
        if len(lines) < 2:
            print("Pose file missing positions.")
            return
        values = lines[1].strip().split()
        if len(values) < 4:
            print("First pose line missing values.")
            return
        x1, y1, z1 = map(float, values[1+5*3:4+5*3])
        x2, y2, z2 = map(float, values[1+6*3:4+6*3])
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2
        z = (z1 + z2) / 2
        print(f"Initial position: ({x:.2f}, {y:.2f}, {z:.2f})")

if __name__ == "__main__":
    args = parse_args()
    print_initial_position(args.pose_path)
