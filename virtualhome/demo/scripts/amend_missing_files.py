import os
import argparse
import re
from glob import glob

SUFFIXES = ['normal', 'seg_class', 'seg_inst']

def extract_index(fname, suffix):
    """
    Extract numeric index from filename like: Action_0420_0_seg_inst.png
    """
    pattern = re.compile(r'^(.+?)_(\d+)_0_' + re.escape(suffix) + r'\.png$')
    match = pattern.match(os.path.basename(fname))
    if not match:
        return None
    return int(match.group(2))  # returns 420 as int

def check_gaps(folder, suffix):
    paths = glob(os.path.join(folder, f"*_{suffix}.png"))
    indices = sorted(filter(None, [extract_index(p, suffix) for p in paths]))
    
    if not indices:
        print(f"{suffix:<10}: ❓ no frames found")
        return

    missing = [i for i in range(indices[0], indices[-1] + 1) if i not in indices]
    if not missing:
        print(f"{suffix:<10}: ✅ continuous [{indices[0]} → {indices[-1]}] ({len(indices)} frames)")
    else:
        print(f"{suffix:<10}: ❌ missing frames: {missing}")
        
import os
import shutil
from typing import List

def fill_missing_frames(folder: str, suffixes: List[str], prefix: str = "Action", depth_ext: str = ".exr"):
    """
    For each suffix in ['normal', 'seg_class', 'seg_inst', 'depth'], find missing frames.
    If all suffixes (including depth) are missing the same frame, fill them by copying the previous frame.
    """
    def get_existing_indices(suffix, ext=".png"):
        files = os.listdir(folder)
        indices = []
        for f in files:
            if f.endswith(f"{suffix}{ext}") and f.startswith(prefix):
                try:
                    idx = int(f.split('_')[1])
                    indices.append(idx)
                except:
                    pass
        return sorted(indices)
    
    image_ext = ".png"
    all_indices = [get_existing_indices(suffix, image_ext) for suffix in suffixes]
    depth_indices = get_existing_indices("depth", depth_ext)

    # Check all lists agree
    if not all(indices == all_indices[0] for indices in all_indices[1:] + [depth_indices]):
        raise ValueError("Mismatch between available frame indices across different image modes!")

    existing = all_indices[0]
    min_id, max_id = existing[0], existing[-1]
    missing = [i for i in range(min_id, max_id + 1) if i not in existing]

    for idx in missing:
        src_idx = idx - 1
        if src_idx not in existing:
            raise RuntimeError(f"Cannot fill missing frame {idx} because source {src_idx} is missing.")
        
        for suffix in suffixes:
            src = os.path.join(folder, f"{prefix}_{src_idx:04d}_0_{suffix}{image_ext}")
            dst = os.path.join(folder, f"{prefix}_{idx:04d}_0_{suffix}{image_ext}")
            shutil.copyfile(src, dst)

        src_depth = os.path.join(folder, f"{prefix}_{src_idx:04d}_0_depth{depth_ext}")
        dst_depth = os.path.join(folder, f"{prefix}_{idx:04d}_0_depth{depth_ext}")
        shutil.copyfile(src_depth, dst_depth)

    return missing


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=str, required=True, help="Path to frame folder")
    args = parser.parse_args()

    for suffix in SUFFIXES:
        check_gaps(args.folder, suffix)
    
    suffixes = ["normal", "seg_class", "seg_inst"]
    missing_filled = fill_missing_frames(args.folder, suffixes)
    print("Filled missing:", missing_filled)
    
    for suffix in SUFFIXES:
        check_gaps(args.folder, suffix)

if __name__ == "__main__":
    main()
