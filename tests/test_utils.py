import os
import time
from uav.utils import retain_recent_logs, should_flat_wall_dodge
from analysis.utils import retain_recent_views


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


def test_should_flat_wall_dodge_threshold():
    assert should_flat_wall_dodge(1.0, 0.2, 5, 5) is True
    # Not enough probe features -> should be False
    assert should_flat_wall_dodge(1.0, 0.2, 3, 5) is False


def test_should_flat_wall_dodge_flow_std_limit():
    # Excessive variance should disable the fallback dodge
    assert should_flat_wall_dodge(1.0, 0.2, 5, 5, flow_std=50.0) is False


def test_retain_recent_views_keeps_latest(tmp_path):
    view_dir = tmp_path / "views"
    view_dir.mkdir()

    now = time.time()
    for i in range(6):
        p = view_dir / f"flight_view_{i}.html"
        p.write_text("data")
        mod_time = now - i
        os.utime(p, (mod_time, mod_time))

    retain_recent_views(str(view_dir), keep=5)
    remaining = sorted(f.name for f in view_dir.iterdir())
    assert remaining == [f"flight_view_{i}.html" for i in range(5)]


def test_retain_recent_views_missing_dir(tmp_path):
    missing = tmp_path / "missing"
    retain_recent_views(str(missing), keep=5)
    assert not missing.exists()

