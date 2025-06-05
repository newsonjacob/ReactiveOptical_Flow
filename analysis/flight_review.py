"""Enhanced flight review helpers."""

from __future__ import annotations

from typing import Dict, Any, List, Sequence

import numpy as np
import pandas as pd
import glob
import os
import subprocess
import argparse


def _repeated_state_runs(states: Sequence[str]) -> Dict[str, int]:
    """Return longest consecutive run length for each state."""
    runs: Dict[str, int] = {}
    if not states:
        return runs
    prev = states[0]
    count = 1
    for state in states[1:]:
        if state == prev:
            count += 1
        else:
            if count > 1:
                runs[prev] = max(runs.get(prev, 0), count)
            prev = state
            count = 1
    if count > 1:
        runs[prev] = max(runs.get(prev, 0), count)
    return runs


def parse_log(path: str) -> Dict[str, Any]:
    """Parse a single ``full_log_*.csv`` file and compute statistics.

    Returns a dictionary with keys ``frames``, ``collisions``, ``fps_avg``,
    ``loop_avg``, ``distance``, ``start`` and ``end``.
    """
    df = pd.read_csv(path)

    frames = len(df)
    if "collided" in df.columns:
        collisions = int((df["collided"] > 0).sum())
    else:
        collisions = 0

    fps_avg = float(df["fps"].mean()) if "fps" in df.columns else float("nan")
    loop_avg = float(df["loop_s"].mean()) if "loop_s" in df.columns else float("nan")

    start = df.loc[0, ["pos_x", "pos_y", "pos_z"]].to_numpy()
    end = df.loc[df.index[-1], ["pos_x", "pos_y", "pos_z"]].to_numpy()
    distance = float(np.linalg.norm(end - start))

    states = df["state"].value_counts().to_dict() if "state" in df.columns else {}
    repeats = (
        _repeated_state_runs(df["state"].tolist()) if "state" in df.columns else {}
    )

    return {
        "frames": frames,
        "collisions": collisions,
        "fps_avg": fps_avg,
        "loop_avg": loop_avg,
        "distance": distance,
        "start": start,
        "end": end,
        "states": states,
        "repeats": repeats,
    }


def align_path(path: np.ndarray, obstacles: List[dict], scale: float = 1.0) -> np.ndarray:
    """Return the UAV path aligned to obstacle coordinates."""
    from .visualize_flight import find_alignment_marker, compute_offset

    marker = find_alignment_marker(obstacles)
    offset = compute_offset(path[0], marker, scale=scale)

    scaled = path * scale
    flipped = np.column_stack((scaled[:, 0], -scaled[:, 1], scaled[:, 2]))
    aligned = flipped + offset
    return aligned


def review_run(log_path: str, obstacles_path: str = "analysis/obstacles.json") -> Dict[str, Any]:
    """Compile statistics and alignment info for a single run."""
    from .visualize_flight import load_obstacles, load_telemetry

    stats = parse_log(log_path)
    obstacles = load_obstacles(obstacles_path)
    telemetry, *_ = load_telemetry(log_path)
    aligned = align_path(telemetry, obstacles)

    stats["aligned_start"] = aligned[0]
    stats["aligned_end"] = aligned[-1]

    return stats


def get_recent_logs(log_dir: str = "flow_logs", limit: int = 5) -> List[str]:
    """Return up to ``limit`` most recent ``full_log_*.csv`` files."""
    pattern = os.path.join(log_dir, "full_log_*.csv")
    logs = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
    return logs[:limit]


def ensure_visualization(log_path: str) -> str:
    """Ensure an HTML visualization exists for ``log_path``."""
    ts = os.path.basename(log_path)[len("full_log_"):-len(".csv")]
    html = os.path.join("analysis", f"flight_view_{ts}.html")
    if not os.path.exists(html):
        cmd = [
            "python",
            os.path.join("analysis", "visualize_flight.py"),
            "--log",
            log_path,
            "--obstacles",
            os.path.join("analysis", "obstacles.json"),
            "--output",
            html,
        ]
        subprocess.run(cmd, check=False)
    return html


def generate_report(log_paths: Sequence[str], obstacles_path: str = "analysis/obstacles.json") -> str:
    """Return a text summary for the given log files."""
    lines = []
    for path in log_paths:
        stats = review_run(path, obstacles_path)
        start = " ".join(f"{v:.2f}" for v in stats["aligned_start"])
        end = " ".join(f"{v:.2f}" for v in stats["aligned_end"])
        repeats = ", ".join(f"{k}:{v}" for k, v in stats["repeats"].items()) or "none"
        line = (
            f"{os.path.basename(path)}: frames={stats['frames']} collisions={stats['collisions']} "
            f"start=[{start}] end=[{end}] repeats={repeats}"
        )
        lines.append(line)
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Review recent UAV flights")
    parser.add_argument("--limit", type=int, default=3, help="Number of recent logs to process")
    parser.add_argument("--log-dir", default="flow_logs", help="Directory containing logs")
    args = parser.parse_args()

    logs = get_recent_logs(args.log_dir, limit=args.limit)
    if not logs:
        print(f"No log files found in {args.log_dir}")
        return

    for log in logs:
        ensure_visualization(log)

    report = generate_report(logs, obstacles_path=os.path.join("analysis", "obstacles.json"))
    report_path = os.path.join("analysis", "flight_review_report.txt")
    with open(report_path, "w") as fh:
        fh.write(report + "\n")

    print(report)
    print(f"Report saved to {report_path}")


if __name__ == "__main__":
    main()
