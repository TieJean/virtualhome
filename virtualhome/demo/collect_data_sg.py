import os, json, argparse
import sys
from copy import deepcopy
from typing import Dict, Any, Iterable, Tuple, List, Set
from tqdm import tqdm

sys.path.append('../simulation')
from utils_demo import *
from graph_utils import *

# ────────────────────────────────────────────────────────────────────────────────
# Args
# ────────────────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(description="Post-process collected data for VirtualHome.")
    parser.add_argument('--data_dir', type=str, default="../../unity_output",
                        help='Root directory containing scene*_* folders.')
    parser.add_argument('--scene_ids', nargs='+', type=int, required=True,
                        help='List of scene IDs to process (e.g., 4 7 12).')
    parser.add_argument('--reindex', action='store_true',
                        help='If set, compact node IDs when pruning.')
    return parser.parse_args()

# ────────────────────────────────────────────────────────────────────────────────
# Prune
# ────────────────────────────────────────────────────────────────────────────────

def prune_graph(graph: Dict[str, Any],
                keep_ids: Iterable[int],
                reindex: bool = False) -> Tuple[Dict[str, Any], Dict[int, int]]:
    """
    Keep only nodes whose id ∈ keep_ids and edges whose endpoints are both kept.
    Optionally reindex node ids to 0..N-1.
    """
    keep = set(keep_ids)
    pruned = {k: deepcopy(v) for k, v in graph.items() if k not in ("nodes", "edges")}
    kept_nodes = [deepcopy(n) for n in graph.get("nodes", []) if isinstance(n, dict) and n.get("id") in keep]

    if reindex:
        start_id = 0
        id_map = {n["id"]: i + start_id for i, n in enumerate(kept_nodes)}
        for n in kept_nodes:
            n["id"] = id_map[n["id"]]
    else:
        id_map = {n["id"]: n["id"] for n in kept_nodes}

    kept_edges = []
    for e in graph.get("edges", []):
        if not isinstance(e, dict):
            continue
        u, v = e.get("from_id"), e.get("to_id")
        if u in keep and v in keep:
            e2 = deepcopy(e)
            if reindex:
                e2["from_id"] = id_map[u]
                e2["to_id"]   = id_map[v]
            kept_edges.append(e2)

    pruned["nodes"] = kept_nodes
    pruned["edges"] = kept_edges
    return pruned, id_map

# ────────────────────────────────────────────────────────────────────────────────
# I/O helpers & validation
# ────────────────────────────────────────────────────────────────────────────────

def _files_exist(dataname_dir: str) -> bool:
    return (
        os.path.isfile(os.path.join(dataname_dir, "0", "gt_annotations.json"))
        and os.path.isfile(os.path.join(dataname_dir, "0", "graph.json"))
    )

def _quick_validate(dataname_dir: str) -> bool:
    """
    Avoids later runtime warnings by checking structure:
    - annotations: list of dicts, each having list 'frame_nodes' (or empty list ok)
    - graph: dict with 'nodes' (list of dicts) and 'edges' (list)
    """
    try:
        with open(os.path.join(dataname_dir, "0", "gt_annotations.json"), "r") as f:
            annotations = json.load(f)
        if not isinstance(annotations, list):
            return False
        for a in annotations[:3]:  # sample a few; large files okay
            if not isinstance(a, dict):
                return False
            fn = a.get("frame_nodes", [])
            if fn is not None and not isinstance(fn, list):
                return False

        with open(os.path.join(dataname_dir, "0", "graph.json"), "r") as f:
            graph = json.load(f)
        if not isinstance(graph, dict):
            return False
        if not isinstance(graph.get("nodes", []), list):
            return False
        # nodes should be dict-like with 'id'
        for n in graph.get("nodes", [])[:3]:
            if not isinstance(n, dict):
                return False
            if "id" not in n:
                return False
        if not isinstance(graph.get("edges", []), list):
            return False
        return True
    except Exception:
        return False

def discover_datanames(root: str, scene_ids: Iterable[int]) -> List[str]:
    """
    Under `root`, find directories whose names start with `scene{scene_id}_`.
    Only return those that contain required files AND pass quick validation.
    """
    if not os.path.isdir(root):
        raise NotADirectoryError(f"data_dir does not exist: {root}")

    prefixes = {f"scene{sid}_" for sid in scene_ids}
    candidates: List[str] = []
    for entry in os.listdir(root):
        full = os.path.join(root, entry)
        if not os.path.isdir(full):
            continue
        if any(entry.startswith(p) for p in prefixes):
            # require files + quick validation to suppress later warnings
            if _files_exist(full) and _quick_validate(full):
                candidates.append(entry)

    def _sort_key(name: str):
        # e.g., "scene4_00", "scene4_00_extra", "scene4_12"
        try:
            base = name.split("_")
            sid = int(base[0].replace("scene", ""))
            sub = base[1] if len(base) > 1 else ""
            subnum = int(sub) if sub.isdigit() else 10_000
            return (sid, subnum, name)
        except Exception:
            return (10**9, 10**9, name)

    return sorted(candidates, key=_sort_key)

# ────────────────────────────────────────────────────────────────────────────────
# Core processing
# ────────────────────────────────────────────────────────────────────────────────

def generate_sg(data_dir: str, reindex: bool = False):
    ann_path  = os.path.join(data_dir, "0", "gt_annotations.json")
    graph_path = os.path.join(data_dir, "0", "graph.json")

    with open(ann_path, 'r') as f:
        annotations = json.load(f)
    if not annotations:
        raise ValueError(f"No annotations found in {ann_path}")

    with open(graph_path, 'r') as f:
        graph = json.load(f)
    if not graph:
        raise ValueError(f"No graph found in {graph_path}")

    visible_node_ids = set()
    for annotation in annotations:
        if not isinstance(annotation, dict):
            continue
        frame_nodes = annotation.get("frame_nodes", []) or []
        for x in frame_nodes:
            if isinstance(x, dict) and "id" in x:
                visible_node_ids.add(x["id"])

    pruned_graph, _ = prune_graph(graph, visible_node_ids, reindex=reindex)
    savepath = os.path.join(data_dir, "0", "pruned_graph.json")
    with open(savepath, 'w') as f:
        json.dump(pruned_graph, f)

def run(args):
    datanames = discover_datanames(args.data_dir, args.scene_ids)
    if not datanames:
        raise ValueError(f"No valid datanames found in {args.data_dir} for scene_ids={args.scene_ids}")

    for dataname in tqdm(datanames, desc="Processing datanames"):
        data_dir = os.path.join(args.data_dir, dataname)
        # generate_sg should now be clean since we pre-validated
        generate_sg(data_dir, reindex=args.reindex)

if __name__ == "__main__":
    args = parse_args()
    run(args)
