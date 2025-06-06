import unittest.mock as mock
import time
from tests.conftest import airsim_stub

from uav.navigation import Navigator


class DummyFuture:
    def __init__(self):
        self.join_called = False

    def join(self):
        self.join_called = True


class DummyClient:
    def __init__(self):
        self.moveByVelocityAsync = mock.MagicMock(side_effect=self._moveByVelocityAsync)
        self.moveByVelocityBodyFrameAsync = mock.MagicMock(side_effect=self._moveByVelocityBodyFrameAsync)
        self.calls = []

    def _moveByVelocityAsync(self, *args, **kwargs):
        fut = DummyFuture()
        self.calls.append(('moveByVelocityAsync', args, kwargs, fut))
        return fut

    def _moveByVelocityBodyFrameAsync(self, *args, **kwargs):
        fut = DummyFuture()
        self.calls.append(('moveByVelocityBodyFrameAsync', args, kwargs, fut))
        return fut


def test_brake_updates_flags_and_calls():
    client = DummyClient()
    nav = Navigator(client)
    prev = nav.last_movement_time
    result = nav.brake()
    assert result == 'brake'
    assert nav.braked is True
    assert nav.dodging is False
    assert nav.last_movement_time == prev
    name, args, kwargs, fut = client.calls[-1]
    assert name == 'moveByVelocityAsync'
    assert args == (0, 0, 0, 1)
    assert fut.join_called is True


def test_dodge_left_sets_flags_and_calls():
    client = DummyClient()
    nav = Navigator(client)
    prev = nav.last_movement_time
    result = nav.dodge(0, 0, 20)
    assert result == 'dodge_left'
    assert nav.braked is False
    assert nav.dodging is True
    assert nav.last_movement_time > prev
    assert client.moveByVelocityBodyFrameAsync.call_count == 2
    call1 = client.moveByVelocityBodyFrameAsync.call_args_list[0]
    assert call1.args == (0, 0, 0, 0.2)
    call2 = client.moveByVelocityBodyFrameAsync.call_args_list[1]
    assert call2.args == (0.3, -1.0, 0, 2.0)
    fut1 = client.calls[0][3]
    fut2 = client.calls[1][3]
    assert fut1.join_called is True
    assert fut2.join_called is True


def test_ambiguous_dodge_forces_lower_flow_side():
    client = DummyClient()
    nav = Navigator(client)
    result = nav.dodge(10, 10.5, 11)
    assert result == 'dodge_left'
    assert client.moveByVelocityBodyFrameAsync.call_count == 2
    call1 = client.moveByVelocityBodyFrameAsync.call_args_list[0]
    assert call1.args == (0, 0, 0, 0.2)
    call2 = client.moveByVelocityBodyFrameAsync.call_args_list[1]
    assert call2.args == (0.0, -1.0, 0, 2.0)


def test_resume_forward_clears_flags_and_calls():
    client = DummyClient()
    nav = Navigator(client)
    prev = nav.last_movement_time
    result = nav.resume_forward()
    assert result == 'resume'
    assert nav.braked is False
    assert nav.dodging is False
    assert nav.last_movement_time > prev
    client.moveByVelocityAsync.assert_called_once()
    args, kwargs = client.moveByVelocityAsync.call_args
    assert args[:3] == (2, 0, 0)
    assert kwargs.get('duration') == 3
    assert kwargs.get('drivetrain') == airsim_stub.DrivetrainType.ForwardOnly


def test_nudge_updates_time_and_calls():
    client = DummyClient()
    nav = Navigator(client)
    prev = nav.last_movement_time
    result = nav.nudge()
    assert result == 'nudge'
    assert nav.braked is False
    assert nav.dodging is False
    assert nav.last_movement_time > prev
    name, args, kwargs, fut = client.calls[-1]
    assert name == 'moveByVelocityAsync'
    assert args == (0.5, 0, 0, 1)
    assert fut.join_called is True


def test_reinforce_updates_time_and_calls():
    client = DummyClient()
    nav = Navigator(client)
    prev = nav.last_movement_time
    result = nav.reinforce()
    assert result == 'resume_reinforce'
    assert nav.braked is False
    assert nav.dodging is False
    assert nav.last_movement_time > prev
    client.moveByVelocityAsync.assert_called_once()
    args, kwargs = client.moveByVelocityAsync.call_args
    assert args[:3] == (2, 0, 0)
    assert kwargs.get('duration') == 3
    assert kwargs.get('drivetrain') == airsim_stub.DrivetrainType.ForwardOnly


def test_dodge_settle_duration_short():
    client = DummyClient()
    nav = Navigator(client)
    before = time.time()
    nav.dodge(0, 0, 20)
    assert nav.settling is True
    assert nav.settle_end_time - before <= 0.5
