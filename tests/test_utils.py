import os
import time
from uav.utils import retain_recent_logs, retain_recent_views, should_flat_wall_dodge


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


def test_retain_recent_views_keeps_latest(tmp_path):
    view_dir = tmp_path / "views"
    view_dir.mkdir()

    now = time.time()
    for i in range(8):
        p = view_dir / f"flight_view_{i}.html"
        p.write_text("data")
        mod_time = now - i
        os.utime(p, (mod_time, mod_time))

    (view_dir / "other.html").write_text("x")

    retain_recent_views(str(view_dir), keep=5)
    remaining = sorted(f.name for f in view_dir.iterdir())
    expected = [f"flight_view_{i}.html" for i in range(5)] + ["other.html"]
    assert remaining == expected


def test_retain_recent_views_missing_dir(tmp_path):
    missing = tmp_path / "missing"
    retain_recent_views(str(missing), keep=5)
    assert not missing.exists()

