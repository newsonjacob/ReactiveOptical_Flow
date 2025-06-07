import importlib
import sys
import types

import tests.conftest  # ensure stubs loaded for numpy, cv2, etc.
from uav import utils


def test_flow_std_max_matches_utils(monkeypatch):
    # Provide minimal airsim stub so main imports cleanly
    airsim_stub = types.SimpleNamespace(ImageRequest=object, ImageType=object)
    monkeypatch.setitem(sys.modules, 'airsim', airsim_stub)
    # Stub pandas if missing to satisfy analysis imports
    if 'pandas' not in sys.modules:
        monkeypatch.setitem(sys.modules, 'pandas', types.SimpleNamespace())
    main = importlib.import_module('main')
    importlib.reload(main)
    assert main.FLOW_STD_MAX == utils.FLOW_STD_MAX

