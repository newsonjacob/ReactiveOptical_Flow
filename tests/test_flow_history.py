import numpy as np
from uav.perception import FlowHistory


def test_window_size_and_update():
    fh = FlowHistory(size=3)
    fh.update(1, 2, 3)
    fh.update(4, 5, 6)
    fh.update(7, 8, 9)
    assert len(fh.window) == 3

    fh.update(10, 11, 12)
    assert len(fh.window) == 3
    assert np.allclose(fh.window[0], np.array([4, 5, 6]))
    assert np.allclose(fh.window[-1], np.array([10, 11, 12]))


def test_average_values():
    fh = FlowHistory(size=3)
    fh.update(1, 2, 3)
    fh.update(4, 5, 6)
    fh.update(7, 8, 9)
    avg = fh.average()
    assert np.allclose(avg, (4.0, 5.0, 6.0))

    fh.update(10, 11, 12)
    avg = fh.average()
    assert np.allclose(avg, (7.0, 8.0, 9.0))
