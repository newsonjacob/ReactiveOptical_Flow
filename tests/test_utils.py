import os
import time
from uav.utils import retain_recent_logs, retain_recent_views


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


def test_retain_recent_views_keeps_latest(tmp_path):
    analysis_dir = tmp_path / "analysis"
    analysis_dir.mkdir()

    now = time.time()
    for i in range(6):
        p = analysis_dir / f"flight_view_{i}.html"
        p.write_text("html")
        mod_time = now - i
        os.utime(p, (mod_time, mod_time))

    retain_recent_views(str(analysis_dir), keep=5)
    remaining = sorted(f.name for f in analysis_dir.iterdir())
    assert remaining == [
        "flight_view_0.html",
        "flight_view_1.html",
        "flight_view_2.html",
        "flight_view_3.html",
        "flight_view_4.html",
    ]


def test_retain_recent_views_missing_dir(tmp_path):
    missing = tmp_path / "missing"
    retain_recent_views(str(missing), keep=5)
    assert not missing.exists()

