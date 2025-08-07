import argparse
import matplotlib.pyplot as plt
import cv2

import sys
sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose_path", type=str, required=True)
    parser.add_argument("--scene_id", type=int, required=True)
    parser.add_argument("--output_path", type=str, default="../../outputs/trajectory.png", help="Output path for the visualization")
    parser.add_argument("--port", type=str, required=True, help="Port for the Unity simulator")
    parser.add_argument("--scale", type=float, default=19.0, help="Scale factor for trajectory coordinates")
    parser.add_argument("--offset_x", type=float, default=0.0, help="X offset for trajectory starting point")
    parser.add_argument("--offset_z", type=float, default=0.0, help="Z offset for trajectory starting point")
    parser.add_argument("--flip_x", action="store_true", help="Flip trajectory horizontally (negate X coordinates)")
    parser.add_argument("--flip_z", action="store_true", help="Flip trajectory vertically (negate Z coordinates)")
    return parser.parse_args()

def visualize_trajectory(args):
    # Marker size parameters for easy adjustment
    line_width = 2.0
    point_size = 25
    start_end_size = 80
    start_end_linewidth = 2
    
    positions = []
    with open(args.pose_path, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            values = line.strip().split()
            if len(values) < 4:
                continue
            x1, y1, z1 = map(float, values[1+5*3:4+5*3])
            x2, y2, z2 = map(float, values[1+6*3:4+6*3])
            positions.append([(x1+x2)/2, (y1+y2)/2, ((z1+z2)/2)])
    
    # Extract x,z coordinates and apply transformations
    x_coords = []
    z_coords = []
    for pos in positions:
        x, y, z = pos
        # Apply scale and offset transformations
        x_transformed = x * args.scale + args.offset_x
        z_transformed = z * args.scale + args.offset_z
        
        # Apply horizontal flip if requested
        if args.flip_x:
            x_transformed = -x_transformed
            
        # Apply vertical flip if requested
        if args.flip_z:
            z_transformed = -z_transformed
            
        x_coords.append(x_transformed)
        z_coords.append(z_transformed)
    
    # Print transformation info for debugging
    print(f"Applied transformations:")
    print(f"  Scale: {args.scale}")
    print(f"  Offset X: {args.offset_x}")
    print(f"  Offset Z: {args.offset_z}")
    print(f"  Flip X: {args.flip_x}")
    print(f"  Flip Z: {args.flip_z}")
    print(f"  Original range X: [{min([pos[0] for pos in positions]):.3f}, {max([pos[0] for pos in positions]):.3f}]")
    print(f"  Original range Z: [{min([pos[2] for pos in positions]):.3f}, {max([pos[2] for pos in positions]):.3f}]")
    print(f"  Transformed range X: [{min(x_coords):.3f}, {max(x_coords):.3f}]")
    print(f"  Transformed range Z: [{min(z_coords):.3f}, {max(z_coords):.3f}]")
    
    # Create visualization with Unity scene as background
    plt.figure(figsize=(12, 10))
    
    # Display the Unity scene image as background
    if hasattr(args, 'image') and args.image is not None:
        # Convert BGR to RGB for matplotlib
        image_rgb = cv2.cvtColor(args.image, cv2.COLOR_BGR2RGB)
        plt.imshow(image_rgb)
        plt.axis('off')  # Hide axes for cleaner look
    
    # Overlay trajectory on the image
    # Note: You may need to adjust coordinate transformation based on Unity's coordinate system
    # and the image dimensions. This is a basic overlay - you might need to scale/transform coordinates
    
    # Plot trajectory line with better visibility
    plt.plot(x_coords, z_coords, color='#FF4444', linewidth=line_width, alpha=0.95, label='Trajectory', zorder=10)
    
    # Plot trajectory points with better color scheme
    scatter = plt.scatter(x_coords, z_coords, c=range(len(x_coords)), cmap='plasma', s=point_size, alpha=0.9, zorder=11, edgecolors='white', linewidth=0.5)
    
    # Add start and end markers with better contrast
    plt.scatter(x_coords[0], z_coords[0], c='#00FF00', s=start_end_size, marker='o', label='Start', zorder=12, 
                edgecolors='black', linewidth=start_end_linewidth, alpha=0.9)
    plt.scatter(x_coords[-1], z_coords[-1], c='#FF0000', s=start_end_size, marker='s', label='End', zorder=12, 
                edgecolors='black', linewidth=start_end_linewidth, alpha=0.9)
    
    # Add colorbar for trajectory progression
    cbar = plt.colorbar(scatter, ax=plt.gca(), shrink=0.8, pad=0.02)
    cbar.set_label('Trajectory Progress', rotation=270, labelpad=15)
    
    plt.title(f'Trajectory Overlay - Scene {args.scene_id}', fontsize=14, fontweight='bold')
    plt.legend(loc='upper right', fontsize=10)
    
    # Remove grid since we have the scene image
    # plt.grid(True, alpha=0.3)
    
    # Save the overlay image
    plt.savefig(args.output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Trajectory overlay saved to: {args.output_path}")

if __name__ == "__main__":
    args = parse_args()
    args.debug_dir = "../../outputs"
    
    comm = UnityCommunication(port=args.port)
    comm.timeout_wait = 1200 
    comm.reset(args.scene_id)
    comm.remove_terrain()
    
    top_view = get_scene_cameras(comm, [-1])[0]
    args.image = top_view
    
    visualize_trajectory(args)
    
    # python scripts/visualize_trajectory.py --scene_id 4 --pose_path ../../unity_output/scene4_00/0/pd_scene4_00.txt  --port 18080 --scale 19.0  --flip_z --offset_x 280 --offset_z -140
    # python scripts/visualize_trajectory.py --scene_id 10 --pose_path ../../unity_output/scene10_00/0/pd_scene10_00.txt  --port 18080 --flip_z --scale 12.5 --offset_x 335 --offset_z -50
    # python scripts/visualize_trajectory.py --scene_id 15 --pose_path ../../unity_output/scene15_00/0/pd_scene15_00.txt  --port 18080 --flip_z --scale 14.2 --offset_x 410 --offset_z -250