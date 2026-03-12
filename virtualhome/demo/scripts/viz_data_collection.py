# scripts/viz_data_collection.py
#
# Side-by-side: [stitched video | animated trajectory panel]
# Implemented:
#  - Old trajectory style (matches visualize_trajectory.py):
#       * line color #FF4444
#       * plasma colormap points with white edge
#       * start: green circle w/ black edge
#       * end: red square w/ black edge
#  - Remove legend/title/HUD
#  - Replace ugly gray background by sampling pixel (10,10) and whitening within dist<=5
#  - Crop ONLY the middle 1/3 of columns (keep full height)
#  - Scale traj background to match stitched video HEIGHT (keep aspect ratio)
#  - If --right_width > 0, pad/crop background to that width and shift points accordingly

import argparse
import os
import sys
from typing import List, Tuple

import cv2
import numpy as np
from tqdm import tqdm

# Unity top-down render
sys.path.append("../simulation")
from unity_simulator.comm_unity import UnityCommunication  # noqa: E402

sys.path.append(".")
from utils_demo import get_scene_cameras  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scene_id", type=int, required=True)
    p.add_argument("--pose_path", type=str, required=True)
    p.add_argument("--port", type=str, required=True)

    p.add_argument("--scale", type=float, default=19.0)
    p.add_argument("--offset_x", type=float, default=0.0)
    p.add_argument("--offset_z", type=float, default=0.0)
    p.add_argument("--flip_x", action="store_true")
    p.add_argument("--flip_z", action="store_true")

    p.add_argument("--vidpath", type=str, required=True, help="Input stitched video (left panel).")

    p.add_argument("--outpath", type=str, default="", help="Output mp4 path. If empty, derived from --vidpath.")
    p.add_argument(
        "--right_width",
        type=int,
        default=0,
        help="Right panel width in px. 0 => use natural width after height-scaling (recommended).",
    )
    p.add_argument("--max_frames", type=int, default=0, help="If >0, only render first N frames.")

    # Styling (OpenCV pixel sizes)
    p.add_argument("--line_thickness", type=int, default=6)
    p.add_argument("--point_radius", type=int, default=8)
    p.add_argument("--point_edge_thickness", type=int, default=2)
    p.add_argument("--start_end_radius", type=int, default=10)
    p.add_argument("--start_end_edge_thickness", type=int, default=2)

    # Cleaning / cropping knobs
    p.add_argument("--bg_sample_x", type=int, default=10)
    p.add_argument("--bg_sample_y", type=int, default=10)
    p.add_argument("--bg_thresh", type=float, default=5.0, help="Euclidean color distance threshold for whitening.")
    p.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (e.g., 2.0 = 2x faster).")

    return p.parse_args()


def derive_outpath(vidpath: str) -> str:
    base, ext = os.path.splitext(vidpath)
    if ext.lower() not in [".mp4", ".mov", ".mkv", ".avi"]:
        ext = ".mp4"
    return base + "_with_traj.mp4"


def read_positions_from_pose_file(pose_path: str) -> List[Tuple[float, float, float]]:
    positions: List[Tuple[float, float, float]] = []
    with open(pose_path, "r") as f:
        lines = f.readlines()

    for line in lines[1:]:
        values = line.strip().split()
        if len(values) < 4:
            continue
        try:
            x1, y1, z1 = map(float, values[1 + 5 * 3 : 4 + 5 * 3])
            x2, y2, z2 = map(float, values[1 + 6 * 3 : 4 + 6 * 3])
        except Exception:
            continue
        positions.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0))
    return positions


def transform_positions_to_pixels(
    positions: List[Tuple[float, float, float]],
    scale: float,
    offset_x: float,
    offset_z: float,
    flip_x: bool,
    flip_z: bool,
) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for (x, _y, z) in positions:
        xt = x * scale + offset_x
        zt = z * scale + offset_z
        if flip_x:
            xt = -xt
        if flip_z:
            zt = -zt
        out.append((xt, zt))
    return out


def get_unity_top_view(scene_id: int, port: str) -> np.ndarray:
    comm = UnityCommunication(port=port)
    comm.timeout_wait = 1200
    comm.reset(scene_id)
    comm.remove_terrain()
    top_view = get_scene_cameras(comm, [-1])[0]  # BGR
    return top_view


def whiten_background_by_sample(img_bgr: np.ndarray, sample_xy: Tuple[int, int], thresh: float) -> np.ndarray:
    h, w = img_bgr.shape[:2]
    sx, sy = sample_xy
    sx = int(np.clip(sx, 0, w - 1))
    sy = int(np.clip(sy, 0, h - 1))

    sample = img_bgr[sy, sx].astype(np.float32)  # BGR
    img_f = img_bgr.astype(np.float32)
    dist = np.linalg.norm(img_f - sample[None, None, :], axis=2)
    mask = dist <= float(thresh)

    out = img_bgr.copy()
    out[mask] = (255, 255, 255)
    return out


def crop_middle_third_columns(img_bgr: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
    """Crop ONLY the middle 1/3 of columns; keep full height."""
    h, w = img_bgr.shape[:2]
    x0 = w // 3
    x1 = (2 * w) // 3
    y0 = 0
    y1 = h
    cropped = img_bgr[y0:y1, x0:x1].copy()
    return cropped, (x0, y0, x1, y1)


def resize_by_height_keep_aspect(img_bgr: np.ndarray, target_h: int) -> Tuple[np.ndarray, float]:
    """Returns (resized_img, scale=target_h/original_h)."""
    h, w = img_bgr.shape[:2]
    if h <= 0:
        return img_bgr, 1.0
    scale = target_h / float(h)
    new_w = max(1, int(round(w * scale)))
    resized = cv2.resize(img_bgr, (new_w, target_h), interpolation=cv2.INTER_AREA)
    return resized, float(scale)


def pad_or_center_crop_to_width(img_bgr: np.ndarray, target_w: int) -> Tuple[np.ndarray, float]:
    """
    Returns (new_img, x_shift) where x_shift should be ADDED to x coords after height scaling.
      padding => +pad_left
      cropping => -crop_x0
    """
    h, w = img_bgr.shape[:2]
    if w == target_w:
        return img_bgr, 0.0

    if w > target_w:
        crop_x0 = (w - target_w) // 2
        cropped = img_bgr[:, crop_x0 : crop_x0 + target_w].copy()
        return cropped, float(-crop_x0)

    pad_total = target_w - w
    pad_l = pad_total // 2
    pad_r = pad_total - pad_l
    padded = cv2.copyMakeBorder(
        img_bgr,
        0,
        0,
        pad_l,
        pad_r,
        borderType=cv2.BORDER_CONSTANT,
        value=(255, 255, 255),
    )
    return padded, float(pad_l)


def map_points_to_panel(
    points_full: List[Tuple[float, float]],
    crop_box: Tuple[int, int, int, int],
    height_scale: float,
    x_shift: float,
) -> List[Tuple[float, float]]:
    x0, y0, _x1, _y1 = crop_box
    mapped: List[Tuple[float, float]] = []
    for (x, y) in points_full:
        xc = (x - x0) * height_scale + x_shift
        yc = (y - y0) * height_scale
        mapped.append((xc, yc))
    return mapped


def _colormap_bgr(n: int) -> np.ndarray:
    """plasma-like via OpenCV applyColorMap (matches old 'plasma' feel)."""
    if n <= 0:
        return np.zeros((0, 3), dtype=np.uint8)
    xs = np.linspace(0, 255, n, dtype=np.uint8).reshape(-1, 1)
    cm = cv2.COLORMAP_PLASMA if hasattr(cv2, "COLORMAP_PLASMA") else cv2.COLORMAP_JET
    return cv2.applyColorMap(xs, cm).reshape(-1, 3)  # BGR


def draw_trajectory_panel(
    bg_bgr: np.ndarray,
    points_px: List[Tuple[float, float]],
    upto_idx: int,
    colors_bgr: np.ndarray,
    line_thickness: int,
    point_radius: int,
    point_edge_thickness: int,
    start_end_radius: int,
    start_end_edge_thickness: int,
) -> np.ndarray:
    """
    Old look:
      - red line (#FF4444)
      - plasma points w/ white edge
      - start: green circle w/ black edge
      - end: red square w/ black edge
    No HUD/legend/title.
    """
    canvas = bg_bgr.copy()
    if not points_px:
        return canvas

    n = len(points_px)
    upto_idx = max(0, min(upto_idx, n - 1))

    pts = np.array(points_px[: upto_idx + 1], dtype=np.float32)
    pts_i = np.round(pts).astype(np.int32)
    pts_i[:, 0] = np.clip(pts_i[:, 0], 0, canvas.shape[1] - 1)
    pts_i[:, 1] = np.clip(pts_i[:, 1], 0, canvas.shape[0] - 1)

    # Line color '#FF4444' in BGR = (0x44,0x44,0xFF) = (68,68,255)
    line_bgr = (68, 68, 255)

    if len(pts_i) >= 2:
        poly = pts_i.reshape(-1, 1, 2)
        cv2.polylines(
            canvas,
            [poly],
            isClosed=False,
            color=line_bgr,
            thickness=line_thickness,
            lineType=cv2.LINE_AA,
        )

    # Points: filled w/ plasma color + white edge
    for i, (x, y) in enumerate(pts_i):
        c = tuple(int(v) for v in colors_bgr[i])  # BGR
        cv2.circle(canvas, (int(x), int(y)), point_radius, c, -1, lineType=cv2.LINE_AA)
        if point_edge_thickness > 0:
            cv2.circle(canvas, (int(x), int(y)), point_radius, (255, 255, 255), point_edge_thickness, lineType=cv2.LINE_AA)

    # Start marker: green circle w/ black edge
    sx, sy = pts_i[0]
    cv2.circle(canvas, (int(sx), int(sy)), start_end_radius, (0, 255, 0), -1, lineType=cv2.LINE_AA)
    cv2.circle(canvas, (int(sx), int(sy)), start_end_radius, (0, 0, 0), start_end_edge_thickness, lineType=cv2.LINE_AA)

    # End marker: red square w/ black edge (always show true end)
    ex, ey = np.round(np.array(points_px[-1], dtype=np.float32)).astype(np.int32)
    ex = int(np.clip(ex, 0, canvas.shape[1] - 1))
    ey = int(np.clip(ey, 0, canvas.shape[0] - 1))
    s = start_end_radius
    cv2.rectangle(canvas, (ex - s, ey - s), (ex + s, ey + s), (0, 0, 255), -1, lineType=cv2.LINE_AA)
    cv2.rectangle(canvas, (ex - s, ey - s), (ex + s, ey + s), (0, 0, 0), start_end_edge_thickness, lineType=cv2.LINE_AA)

    return canvas


def main():
    args = parse_args()

    if not os.path.isfile(args.vidpath):
        raise FileNotFoundError(f"--vidpath not found: {args.vidpath}")
    if not os.path.isfile(args.pose_path):
        raise FileNotFoundError(f"--pose_path not found: {args.pose_path}")

    outpath = args.outpath.strip() or derive_outpath(args.vidpath)
    os.makedirs(os.path.dirname(outpath) or ".", exist_ok=True)

    # --- Open input video ---
    cap = cv2.VideoCapture(args.vidpath)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {args.vidpath}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 1e-6:
        fps = 30.0

    left_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    left_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if args.max_frames > 0:
        total_frames = min(total_frames, args.max_frames)

    right_h = left_h

    # --- Unity top view background ---
    top_view = get_unity_top_view(args.scene_id, args.port)

    cleaned = whiten_background_by_sample(
        top_view,
        sample_xy=(args.bg_sample_x, args.bg_sample_y),
        thresh=args.bg_thresh,
    )

    cropped, crop_box = crop_middle_third_columns(cleaned)

    bg_right, height_scale = resize_by_height_keep_aspect(cropped, right_h)

    x_shift = 0.0
    if args.right_width > 0:
        bg_right, x_shift = pad_or_center_crop_to_width(bg_right, args.right_width)

    right_w = bg_right.shape[1]

    # --- Poses ---
    positions = read_positions_from_pose_file(args.pose_path)
    points_full = transform_positions_to_pixels(
        positions,
        scale=args.scale,
        offset_x=args.offset_x,
        offset_z=args.offset_z,
        flip_x=args.flip_x,
        flip_z=args.flip_z,
    )

    points_panel = map_points_to_panel(points_full, crop_box, height_scale, x_shift)

    # Align to video frames (assumption)
    if total_frames <= 0:
        total_frames = len(points_panel) if len(points_panel) > 0 else 1

    if len(points_panel) == 0:
        print("[Warning] No pose points parsed; trajectory panel will be background-only.")
        colors = _colormap_bgr(0)
    else:
        if len(points_panel) < total_frames:
            last = points_panel[-1]
            points_panel = points_panel + [last] * (total_frames - len(points_panel))
        else:
            points_panel = points_panel[:total_frames]
        colors = _colormap_bgr(len(points_panel))

    # --- Writer ---
    out_w = left_w + right_w
    out_h = left_h
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out_fps = fps * args.speed
    writer = cv2.VideoWriter(outpath, fourcc, out_fps, (out_w, out_h))

    print(f"Input video:  {args.vidpath} ({left_w}x{left_h}, fps={fps:.2f}, frames={total_frames})")
    print(f"Output video: {outpath} ({out_w}x{out_h})")
    print(f"BG sample=({args.bg_sample_x},{args.bg_sample_y}), thresh={args.bg_thresh}")
    print(f"Crop_box={crop_box}, height_scale={height_scale:.4f}, x_shift={x_shift:.1f}, right_w={right_w}")

    frame_idx = 0
    pbar = tqdm(total=total_frames, desc="Rendering side-by-side", unit="frame")
    try:
        while frame_idx < total_frames:
            ret, frame = cap.read()
            if not ret or frame is None:
                break

            traj_panel = draw_trajectory_panel(
                bg_bgr=bg_right,
                points_px=points_panel,
                upto_idx=frame_idx,
                colors_bgr=colors,
                line_thickness=args.line_thickness,
                point_radius=args.point_radius,
                point_edge_thickness=args.point_edge_thickness,
                start_end_radius=args.start_end_radius,
                start_end_edge_thickness=args.start_end_edge_thickness,
            )

            combined = np.hstack([frame, traj_panel])
            writer.write(combined)

            frame_idx += 1
            pbar.update(1)

    finally:
        pbar.close()
        cap.release()
        writer.release()

    print(f"Done. Saved: {outpath}")


if __name__ == "__main__":
    main()

# python scripts/viz_data_collection.py   --scene_id 4   --pose_path ../../unity_output/scene4_00/0/pd_scene4_00.txt   --port 18080   --scale 19.0   --flip_z   --offset_x 280   --offset_z -140   --vidpath ../../outputs/stitched_gallery.mp4   --outpath ../../outputs/stiched_gallery_with_traj.mp4 --speed 2