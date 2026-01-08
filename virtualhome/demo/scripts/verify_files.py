import argparse
import os
import re
import json
import glob

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--root_dir", required=True, help="Path to the root directory containing scene folders")
    p.add_argument("--scene_ids", nargs="+", type=int, required=True, help="List of scene IDs to process")
    p.add_argument("--captioners", nargs="+", default=["gpt4o", "molmo"], help="List of captioners to check (default: gpt4o, molmo)")
    return p.parse_args()

def folder_matches_sid(folder: str, sid: int) -> bool:
    # match: scene{sid}_NN or scene{sid}_NN_* (NN = two digits)
    pat = re.compile(rf"^scene{sid}_(\d{{2}})(?:$|_)")
    return bool(pat.match(folder))

def get_json_length(filepath):
    """Reads a JSON list and returns its length."""
    if not os.path.exists(filepath):
        return None
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
            if isinstance(data, list):
                return len(data)
            return 0 # Handle case where json is not a list
    except Exception:
        return None

def get_position_file_length(filepath):
    """Reads the position txt file, counts lines, checks for header."""
    if not os.path.exists(filepath):
        return None
    try:
        with open(filepath, 'r') as f:
            lines = [l.strip() for l in f.readlines() if l.strip()]
            # Subtract 1 for the header row
            return max(0, len(lines) - 1)
    except Exception:
        return None

def get_perception_frame_count(target_dir):
    """
    Counts perception frames. 
    Uses one modality (e.g., '_normal.png') as an anchor to determine the frame count.
    """
    if not os.path.isdir(target_dir):
        return None
    
    # We look for files ending in _normal.png to count frames.
    # You can change this wildcard to *_depth.exr or others if preferred.
    files = glob.glob(os.path.join(target_dir, "*_normal.png"))
    return len(files)

def main():
    args = parse_args()

    # Dictionary to store missing files or mismatch errors
    # Structure: { "category_name": [list_of_folders] }
    report = {
        "missing_dir": [],
        "missing_pos": [],
        "mismatched_lengths": []
    }
    
    # Initialize lists for dynamic captioner files
    for cap in args.captioners:
        report[f"missing_{cap}_gt"] = []
        report[f"missing_{cap}_nframe1"] = []

    for entry in os.scandir(args.root_dir):
        if not entry.is_dir():
            continue
        folder = entry.name
        
        # Filter by scene_ids
        if not any(folder_matches_sid(folder, sid) for sid in args.scene_ids):
            continue

        # Define paths
        target_dir = os.path.join(args.root_dir, folder, "0")
        
        # 1. Check Directory Existence
        if not os.path.isdir(target_dir):
            report["missing_dir"].append(folder)
            continue

        # 2. Check Position File
        # Pattern: pd_{folder_name}.txt inside the '0' directory
        pos_filename = f"pd_{folder}.txt"
        pos_path = os.path.join(target_dir, pos_filename)
        
        pos_count = get_position_file_length(pos_path)
        if pos_count is None:
            report["missing_pos"].append(folder)

        # 3. Check Perception Files (Anchor Modality)
        perception_count = get_perception_frame_count(target_dir)

        # 4. Check Caption Files (for all requested captioners)
        for cap in args.captioners:
            gt_file = f"caption_{cap}_gt.json"
            nframe1_file = f"caption_{cap}_nframe1.json"
            
            gt_path = os.path.join(target_dir, gt_file)
            nframe1_path = os.path.join(target_dir, nframe1_file)

            # Check GT existence and length
            gt_count = get_json_length(gt_path)
            if gt_count is None:
                report[f"missing_{cap}_gt"].append(folder)
            
            # Check NFrame1 existence and length
            nframe1_count = get_json_length(nframe1_path)
            if nframe1_count is None:
                report[f"missing_{cap}_nframe1"].append(folder)

            # 5. Data Consistency Check
            
            # Check GT Consistency
            if pos_count is not None and perception_count is not None and gt_count is not None:
                if not (pos_count == perception_count == gt_count):
                    error_msg = (f"{folder} ({cap} GT): "
                                 f"Pos={pos_count}, Imgs={perception_count}, JSON={gt_count}")
                    report["mismatched_lengths"].append(error_msg)

            # Check NFrame1 Consistency
            if pos_count is not None and perception_count is not None and nframe1_count is not None:
                if not (pos_count == perception_count == nframe1_count):
                    error_msg = (f"{folder} ({cap} NF1): "
                                 f"Pos={pos_count}, Imgs={perception_count}, JSON={nframe1_count}")
                    report["mismatched_lengths"].append(error_msg)

    # --- Print Report ---
    print("\n--- validation Report ---")
    
    # Sort and print specific issues
    for key in sorted(report.keys()):
        if report[key]:
            report[key].sort()
            print(f"{key}:")
            for item in report[key]:
                print(f"  - {item}")
            print("-" * 20)

    # Summary if clean
    total_issues = sum(len(v) for v in report.values())
    if total_issues == 0:
        print("Success! All scanned folders match criteria and length checks.")

if __name__ == "__main__":
    main()