import os
import time
from uav.utils import retain_recent_logs


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

