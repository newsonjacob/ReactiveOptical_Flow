import os
import time
from uav.utils import retain_recent_logs, compute_probe_mask, DEFAULT_PROBE_HEIGHT


def test_retain_recent_logs_keeps_latest(tmp_path):
    log_dir = tmp_path / "logs"
    log_dir.mkdir()

    now = time.time()
    paths = []
    for i in range(5):
        p = log_dir / f"log_{i}.csv"
        p.write_text("data")
        mod_time = now - i
        os.utime(p, (mod_time, mod_time))
        paths.append(p)

    retain_recent_logs(str(log_dir), keep=3)
    remaining = sorted(f.name for f in log_dir.iterdir())
    assert remaining == ["log_0.csv", "log_1.csv", "log_2.csv"]


def test_retain_recent_logs_missing_dir(tmp_path):
    missing = tmp_path / "missing"
    retain_recent_logs(str(missing), keep=3)
    assert not missing.exists()


def test_compute_probe_mask_default_equivalent():
    y_coords = [0, 50, 120, 170]
    mask = list(compute_probe_mask(y_coords, 300, DEFAULT_PROBE_HEIGHT))
    expected = [y < 300 // 3 for y in y_coords]
    assert mask == expected


def test_compute_probe_mask_custom_fraction():
    y_coords = [0, 140, 160, 260]
    mask = list(compute_probe_mask(y_coords, 400, 0.5))
    expected_threshold = int(400 * 0.5 + 0.5)
    expected = [y < expected_threshold for y in y_coords]
    assert mask == expected

