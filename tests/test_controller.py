import types

from uav.controller import DroneController


class DummyClient:
    pass


class DummyNavigator:
    def __init__(self):
        self.calls = []
        self.braked = False
        self.dodging = False
        self.last_movement_time = 0
        self.grace_period_end_time = 0

    def brake(self):
        self.calls.append('brake')
        return 'brake'

    def blind_forward(self):
        self.calls.append('blind_forward')
        return 'blind_forward'

    def dodge(self, L, C, R, duration=2.0):
        self.calls.append(('dodge', L, C, R))
        return 'dodge'

    def resume_forward(self):
        self.calls.append('resume')
        return 'resume'

    def reinforce(self):
        self.calls.append('reinforce')
        return 'reinforce'

    def nudge(self):
        self.calls.append('nudge')
        return 'nudge'

    def timeout_recover(self):
        self.calls.append('timeout')
        return 'timeout'


def dummy_state(*args, **kwargs):
    pos = types.SimpleNamespace(x_val=0, y_val=0, z_val=0)
    return pos, 0.0, 0.0


def setup_controller(monkeypatch):
    client = DummyClient()
    ctrl = DroneController(client)
    ctrl.navigator = DummyNavigator()
    ctrl.initialize({}, {})
    monkeypatch.setattr('uav.controller.get_drone_state', dummy_state)
    return ctrl


def test_low_feature_brake(monkeypatch):
    ctrl = setup_controller(monkeypatch)
    state = ctrl.decide_action([], 2.0, 0.1, 2.0, time_now=1.0)
    assert state == 'brake'
    assert 'brake' in ctrl.navigator.calls


def test_low_feature_forward(monkeypatch):
    ctrl = setup_controller(monkeypatch)
    state = ctrl.decide_action([], 0.0, 0.0, 0.0, time_now=1.0)
    assert state == 'blind_forward'
    assert 'blind_forward' in ctrl.navigator.calls


def test_high_center_brake(monkeypatch):
    ctrl = setup_controller(monkeypatch)
    points = [[0, 0]] * 10
    state = ctrl.decide_action(points, 0.0, 40.0, 0.0, time_now=1.0)
    assert state == 'brake'
    assert 'brake' in ctrl.navigator.calls
