#!/usr/bin/env python3
import argparse
import errno
import hashlib
import json
import os
import sys
import time
from pathlib import Path

def sha256_file(path: Path, chunk_bytes: int = 8 * 1024 * 1024) -> str:
    h = hashlib.sha256()
    with path.open("rb", buffering=0) as f:
        while True:
            b = f.read(chunk_bytes)
            if not b:
                break
            h.update(b)
    return h.hexdigest()

def safe_stat(path: Path):
    try:
        st = path.stat()
        return {
            "size": st.st_size,
            "mtime": st.st_mtime,
            "mode": oct(st.st_mode),
            "inode": getattr(st, "st_ino", None),
            "dev": getattr(st, "st_dev", None),
        }, None
    except OSError as e:
        return None, {"errno": e.errno, "strerror": e.strerror, "repr": repr(e)}

def test_tmp_write(tmp_dir: Path, bytes_to_write: int = 1024 * 1024):
    """Verify we can create/write/delete a temp file in tmp_dir."""
    tmp_dir.mkdir(parents=True, exist_ok=True)
    test_path = tmp_dir / f".tmp_write_test_{os.getpid()}_{int(time.time())}"
    try:
        with test_path.open("wb") as f:
            f.write(b"\0" * bytes_to_write)
        test_path.unlink()
        return {"ok": True}
    except OSError as e:
        return {"ok": False, "errno": e.errno, "strerror": e.strerror, "repr": repr(e), "tmp_dir": str(tmp_dir)}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", help="Directory to check, e.g. starbench_data/scene.../0/")
    ap.add_argument("--repeat", type=int, default=2, help="How many full read+hash passes per file (default: 2)")
    ap.add_argument("--chunk-mb", type=int, default=8, help="Read chunk size in MB (default: 8)")
    ap.add_argument("--tmp-dir", default=None,
                    help="Directory to test temp write (default: current working directory)")
    ap.add_argument("--output", default="file_check_report.json", help="Output JSON report path")
    ap.add_argument("--only-ext", default=None,
                    help="Comma-separated extensions to check, e.g. '.exr,.png' (default: all)")
    args = ap.parse_args()

    root = Path(args.dir).expanduser().resolve()
    if not root.exists() or not root.is_dir():
        print(f"ERROR: not a directory: {root}", file=sys.stderr)
        sys.exit(2)

    exts = None
    if args.only_ext:
        exts = set([e.strip().lower() for e in args.only_ext.split(",") if e.strip()])

    chunk_bytes = args.chunk_mb * 1024 * 1024
    tmp_dir = Path(args.tmp_dir).expanduser().resolve() if args.tmp_dir else Path.cwd().resolve()

    report = {
        "root": str(root),
        "checked_at_unix": time.time(),
        "repeat": args.repeat,
        "chunk_mb": args.chunk_mb,
        "tmp_write_test": test_tmp_write(tmp_dir),
        "files": [],
        "summary": {},
    }

    # Collect files deterministically
    files = [p for p in root.rglob("*") if p.is_file()]
    files.sort(key=lambda p: str(p))

    problems = 0
    eio_files = 0
    unreadable = 0
    hash_mismatch = 0
    zero_bytes = 0

    for p in files:
        if exts and p.suffix.lower() not in exts:
            continue

        entry = {"path": str(p), "suffix": p.suffix.lower()}
        st, st_err = safe_stat(p)
        entry["stat"] = st
        entry["stat_error"] = st_err

        if st and st["size"] == 0:
            entry["zero_byte"] = True
            zero_bytes += 1

        hashes = []
        read_errors = []

        for i in range(args.repeat):
            try:
                h = sha256_file(p, chunk_bytes=chunk_bytes)
                hashes.append(h)
            except OSError as e:
                err = {"pass": i, "errno": e.errno, "strerror": e.strerror, "repr": repr(e)}
                read_errors.append(err)
                if e.errno == errno.EIO:
                    eio_files += 1
                break  # usually no need to keep trying if we got an OS error

        entry["sha256_passes"] = hashes
        entry["read_errors"] = read_errors

        # Detect inconsistent reads
        if len(hashes) >= 2 and len(set(hashes)) > 1:
            entry["hash_mismatch"] = True
            hash_mismatch += 1

        is_problem = bool(st_err or read_errors or entry.get("hash_mismatch") or entry.get("zero_byte"))
        entry["problem"] = is_problem

        if is_problem:
            problems += 1
            if read_errors:
                unreadable += 1

        report["files"].append(entry)

    report["summary"] = {
        "total_files_seen": len(files),
        "total_files_checked": len(report["files"]),
        "problems": problems,
        "unreadable_or_read_error": unreadable,
        "eio_error_count": eio_files,
        "hash_mismatch_count": hash_mismatch,
        "zero_byte_count": zero_bytes,
    }

    out = Path(args.output).expanduser().resolve()
    with out.open("w") as f:
        json.dump(report, f, indent=2)

    print(f"Wrote report: {out}")
    print("Summary:", json.dumps(report["summary"], indent=2))

    # Print the top suspicious ones for convenience
    bad = [x for x in report["files"] if x.get("problem")]
    if bad:
        print("\nFirst 20 problematic files:")
        for x in bad[:20]:
            print("-", x["path"])
            if x.get("stat_error"):
                print("   stat_error:", x["stat_error"])
            if x.get("read_errors"):
                print("   read_errors:", x["read_errors"])
            if x.get("hash_mismatch"):
                print("   hash_mismatch:", x["sha256_passes"])
            if x.get("zero_byte"):
                print("   zero_byte: True")

if __name__ == "__main__":
    main()
