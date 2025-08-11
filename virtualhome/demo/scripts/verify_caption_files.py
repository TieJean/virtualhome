import argparse
import os
import re

GT_FILE = "caption_gpt4o_gt.json"
NFRAME1_FILE = "caption_gpt4o_nframe1.json"  # change to 'gap4o' if that's not a typo

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--root_dir", required=True)
    p.add_argument("--scene_ids", nargs="+", type=int, required=True)
    return p.parse_args()

def folder_matches_sid(folder: str, sid: int) -> bool:
    # match: scene{sid}_NN or scene{sid}_NN_*   (NN = two digits)
    pat = re.compile(rf"^scene{sid}_(\d{{2}})(?:$|_)")
    return bool(pat.match(folder))

def main():
    args = parse_args()

    missing_gt = []
    missing_nframe1 = []

    for entry in os.scandir(args.root_dir):
        if not entry.is_dir():
            continue
        folder = entry.name
        # include only folders that match any requested sid with the strict pattern
        if not any(folder_matches_sid(folder, sid) for sid in args.scene_ids):
            continue

        target_dir = os.path.join(args.root_dir, folder, "0")
        gt_path = os.path.join(target_dir, GT_FILE)
        nframe1_path = os.path.join(target_dir, NFRAME1_FILE)

        # If the "0" folder itself is missing, count both as missing
        if not os.path.isdir(target_dir):
            missing_gt.append(folder)
            missing_nframe1.append(folder)
            continue

        if not os.path.exists(gt_path):
            missing_gt.append(folder)
        if not os.path.exists(nframe1_path):
            missing_nframe1.append(folder)

    # sort for stable output
    missing_gt.sort()
    missing_nframe1.sort()

    print("Missing caption_gpt4o_gt.json:", missing_gt)
    print("Missing caption_gpt4o_nframe1.json:", missing_nframe1)

if __name__ == "__main__":
    main()
