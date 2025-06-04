import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import types

# Minimal numpy stub for environments without numpy
class DummyArray(list):
    def mean(self, axis=None):
        if axis is None:
            return sum(self) / len(self)
        cols = list(zip(*self))
        return DummyArray([sum(c) / len(c) for c in cols])


def array(obj):
    if isinstance(obj, DummyArray):
        return obj
    return DummyArray(list(obj))


def allclose(a, b, tol=1e-8):
    a_list = list(a)
    b_list = list(b)
    if len(a_list) != len(b_list):
        return False
    return all(abs(x - y) <= tol for x, y in zip(a_list, b_list))


numpy_stub = types.SimpleNamespace(array=array, allclose=allclose)

# Register stub if numpy is missing
if 'numpy' not in sys.modules:
    sys.modules['numpy'] = numpy_stub
# Minimal cv2 stub for environments without OpenCV
if "cv2" not in sys.modules:
    cv2_stub = types.SimpleNamespace(
        createCLAHE=lambda **kwargs: types.SimpleNamespace(apply=lambda img: img),
        goodFeaturesToTrack=lambda *a, **k: None,
        calcOpticalFlowPyrLK=lambda *a, **k: (None, None, None)
    )
    sys.modules["cv2"] = cv2_stub
# Minimal airsim stub for environments without AirSim
class DummyYawMode:
    def __init__(self, is_rate, yaw_or_rate):
        self.is_rate = is_rate
        self.yaw_or_rate = yaw_or_rate

airsim_stub = types.SimpleNamespace(
    DrivetrainType=types.SimpleNamespace(ForwardOnly=1),
    YawMode=DummyYawMode,
    to_eularian_angles=lambda ori: (0, 0, 0),
)
if "airsim" not in sys.modules:
    sys.modules["airsim"] = airsim_stub

