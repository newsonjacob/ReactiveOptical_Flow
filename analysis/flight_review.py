"""Enhanced flight review helpers."""

from __future__ import annotations

from typing import Dict, Any, List

import numpy as np
import pandas as pd


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

    return {
        "frames": frames,
        "collisions": collisions,
        "fps_avg": fps_avg,
        "loop_avg": loop_avg,
        "distance": distance,
        "start": start,
        "end": end,
        "states": states,
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
