import sys

if "numpy" in sys.modules and getattr(sys.modules["numpy"], "__file__", None) is None:
    del sys.modules["numpy"]
if "numpy" in sys.modules:
    real_numpy = sys.modules["numpy"]
else:
    import numpy as real_numpy
    sys.modules["numpy"] = real_numpy
import numpy as np

if "pandas" not in sys.modules:
    import pandas as pd
    sys.modules["pandas"] = pd
else:
    import pandas as pd
import pytest
from analysis.flight_review import parse_log, align_path, generate_report


def test_parse_log_basic(tmp_path):
    data = pd.DataFrame({
        'pos_x': [0, 1, 2],
        'pos_y': [0, 0, 0],
        'pos_z': [0, 0, 0],
        'collided': [0, 1, 0],
        'fps': [10, 20, 30],
        'loop_s': [0.1, 0.2, 0.3],
        'state': ['resume', 'brake', 'resume'],
    })
    log_path = tmp_path / "log.csv"
    data.to_csv(log_path, index=False)

    stats = parse_log(str(log_path))
    assert stats['frames'] == 3
    assert stats['collisions'] == 1
    assert stats['distance'] == np.linalg.norm([2,0,0])
    assert stats['fps_avg'] == 20
    assert stats['loop_avg'] == pytest.approx(0.2)
    assert stats['states']['resume'] == 2
    assert stats['repeats'] == {}


def test_align_path_applies_offset():
    path = np.array([[0, 0, 0], [1, 1, 0]])
    obstacles = [{
        'name': 'PlayerStart_3',
        'location': [5, 5, 0],
        'dimensions': [0, 0, 0],
        'rotation': [0, 0, 0],
    }]

    aligned = align_path(path, obstacles, scale=1.0)
    assert np.allclose(aligned[0], [5, 5, 0])
    assert np.allclose(aligned[1], [6, 4, 0])


def test_parse_log_detects_repeated_states(tmp_path):
    data = pd.DataFrame({
        'pos_x': [0, 0, 0, 0],
        'pos_y': [0, 0, 0, 0],
        'pos_z': [0, 0, 0, 0],
        'state': ['brake', 'brake', 'resume', 'resume'],
    })
    log_path = tmp_path / "log.csv"
    data.to_csv(log_path, index=False)

    stats = parse_log(str(log_path))
    assert stats['repeats'] == {'brake': 2, 'resume': 2}


def test_generate_report(monkeypatch, tmp_path):
    log_path = tmp_path / 'full_log_test.csv'
    df = pd.DataFrame({
        'pos_x': [0, 1],
        'pos_y': [0, 0],
        'pos_z': [0, 0],
        'state': ['resume', 'resume'],
    })
    df.to_csv(log_path, index=False)

    obstacles = [{
        'name': 'PlayerStart_3',
        'location': [0, 0, 0],
        'dimensions': [0, 0, 0],
        'rotation': [0, 0, 0],
    }]

    telemetry = np.array([[0, 0, 0], [1, 0, 0]])

    monkeypatch.setattr('analysis.visualize_flight.load_obstacles', lambda p: obstacles)
    monkeypatch.setattr('analysis.visualize_flight.load_telemetry', lambda p: (telemetry, None, None, None))

    report = generate_report([str(log_path)], obstacles_path='ignored')
    assert 'frames=2' in report
    assert 'repeats=resume:2' in report
