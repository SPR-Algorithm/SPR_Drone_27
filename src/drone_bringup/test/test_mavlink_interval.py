"""Control-message regressions only; these doubles provide no sensor evidence."""
from types import SimpleNamespace
import sys

import pytest

from drone_bringup import mavlink_imu


class ProtocolMessage(SimpleNamespace):
    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return self.source[0]

    def get_srcComponent(self):
        return self.source[1]


def heartbeat(source=(1, 1), armed=False):
    return ProtocolMessage(kind='HEARTBEAT', source=source, autopilot=12,
                           base_mode=128 if armed else 0)


def interval(value, source=(1, 1), message_id=105):
    return ProtocolMessage(kind='MESSAGE_INTERVAL', source=source,
                           message_id=message_id, interval_us=value)


def ack(command, result=0, source=(1, 1)):
    return ProtocolMessage(kind='COMMAND_ACK', source=source, command=command, result=result)


class TestClock:
    __test__ = False

    def __init__(self):
        self.now_ns = 10_000_000_000

    def monotonic_ns(self):
        return self.now_ns

    def monotonic(self):
        return self.now_ns / 1e9

    def time_ns(self):
        return 1_793_279_000_000_000_000 + self.now_ns

    def advance(self, seconds):
        self.now_ns += round(seconds * 1e9)


class ProtocolLink:
    def __init__(self, clock):
        self.clock = clock
        self.message_hooks = []
        self.messages = []
        self.commands = []
        self.on_command = lambda command: None
        self.closed = False
        self.mav = SimpleNamespace(command_long_send=self.command_long_send,
                                   heartbeat_send=lambda *args: None,
                                   timesync_send=lambda *args: None)

    def queue(self, message, delay_s=0):
        self.messages.append((self.clock.now_ns + round(delay_s * 1e9), message))
        self.messages.sort(key=lambda item: item[0])

    def command_long_send(self, system, component, command, confirmation, *params):
        sent = (system, component, command, params[0], params[1])
        self.commands.append(sent)
        self.on_command(sent)

    def recv_match(self, blocking, timeout):
        deadline = self.clock.now_ns + round(timeout * 1e9)
        if self.messages and self.messages[0][0] <= deadline:
            received_at, message = self.messages.pop(0)
            self.clock.now_ns = max(self.clock.now_ns, received_at)
            for hook in self.message_hooks:
                hook(self, message)
            return message
        self.clock.now_ns = deadline
        return None

    def close(self):
        self.closed = True


class ProtocolLock:
    def __init__(self, identity):
        self.identity = identity
        self.closed = False

    def acquire(self):
        return self

    def close(self):
        self.closed = True


@pytest.fixture
def owner(monkeypatch, tmp_path):
    clock = TestClock()
    links, locks, events = [], [], []
    state = SimpleNamespace(next_source=(1, 1), links=links, locks=locks,
                            clock=clock, events=events)

    def connect(*args, **kwargs):
        link = ProtocolLink(clock)
        link.queue(heartbeat(state.next_source))
        links.append(link)
        return link

    def lock(identity):
        result = ProtocolLock(identity)
        locks.append(result)
        return result

    monkeypatch.setitem(sys.modules, 'pymavlink', SimpleNamespace(
        mavutil=SimpleNamespace(mavlink_connection=connect)))
    monkeypatch.setattr(mavlink_imu, 'time', clock)
    monkeypatch.setattr(mavlink_imu, 'DeviceLock', lock)
    monkeypatch.setattr(mavlink_imu, 'verify_serial_free', lambda device: None)
    monkeypatch.setattr(mavlink_imu.importlib.metadata, 'version', lambda name: 'protocol-test-double')
    device = tmp_path / 'serial-endpoint-placeholder'
    device.touch()
    state.session = mavlink_imu.MavlinkImuSession({'device': str(device), 'requested_rate_hz': 200.0})
    state.session.event = lambda event, **fields: events.append((event, fields))
    yield state
    state.session._close_transport(restore=False)


def open_owner(owner):
    owner.session.open()
    return owner.links[-1]


def set_original(owner, link, value, source=(1, 1)):
    link.queue(interval(value, source=source))
    owner.session.poll(timeout_s=0)


def test_delayed_original_response_unlocks_rate_change_once(owner):
    link = open_owner(owner)
    assert link.commands == [(1, 1, 510, 105, 0)]
    owner.clock.advance(0.3)  # The former 200 ms timer must never authorize SET.
    owner.session.poll(timeout_s=0)
    assert len(link.commands) == 1
    assert owner.session.original_interval_us is None
    assert not owner.session.interval_requested

    set_original(owner, link, 20_000)
    assert link.commands[-1] == (1, 1, 511, 105, 5000)
    assert owner.session.original_interval_us == 20_000
    assert owner.session.interval_query_deadline_ns is None
    assert owner.session.interval_requested
    set_original(owner, link, 5000)
    assert len(link.commands) == 2
    assert owner.session.original_interval_us == 20_000


@pytest.mark.parametrize('unrelated', [interval(20_000, source=(2, 1)),
                                      interval(20_000, source=(1, 2)),
                                      interval(20_000, message_id=106),
                                      ack(510), ack(511)])
def test_ack_or_other_source_and_message_cannot_confirm_original(owner, unrelated):
    link = open_owner(owner)
    link.queue(unrelated)
    owner.session.poll(timeout_s=0)
    assert link.commands == [(1, 1, 510, 105, 0)]
    assert owner.session.original_interval_us is None
    assert owner.session.interval_ack is None


def test_missing_original_times_out_without_rate_change_or_default_restore(owner):
    link = open_owner(owner)
    owner.clock.advance(mavlink_imu.INTERVAL_QUERY_TIMEOUT_NS / 1e9)
    with pytest.raises(TimeoutError, match='original HIGHRES_IMU interval'):
        owner.session.poll(timeout_s=0)
    owner.session._close_transport()
    assert link.commands == [(1, 1, 510, 105, 0)]
    assert link.closed and owner.locks[-1].closed
    assert owner.session.interval_query_deadline_ns is None


def test_reply_received_after_deadline_cannot_authorize_rate_change(owner):
    link = open_owner(owner)
    owner.clock.advance(mavlink_imu.INTERVAL_QUERY_TIMEOUT_NS / 1e9 - 0.05)
    link.queue(interval(20_000), delay_s=0.1)
    with pytest.raises(TimeoutError, match='stream rate unchanged'):
        owner.session.poll(timeout_s=0.1)
    assert link.commands == [(1, 1, 510, 105, 0)]


@pytest.mark.parametrize('value', [0, -2])
def test_unavailable_or_invalid_original_never_changes_rate(owner, value):
    link = open_owner(owner)
    with pytest.raises(RuntimeError, match='unavailable or invalid'):
        set_original(owner, link, value)
    owner.session._close_transport()
    assert link.commands == [(1, 1, 510, 105, 0)]
    assert owner.session.original_interval_us is None


def test_rejected_interval_query_never_changes_rate(owner):
    link = open_owner(owner)
    link.queue(ack(510, result=3))
    with pytest.raises(RuntimeError, match='rejected original'):
        owner.session.poll(timeout_s=0)
    owner.session._close_transport()
    assert link.commands == [(1, 1, 510, 105, 0)]


@pytest.mark.parametrize('original', [20_000, -1])
def test_close_restores_exact_confirmed_interval_and_checks_target_readback(owner, original):
    link = open_owner(owner)
    set_original(owner, link, original)

    def reply(command):
        if command[2] == 511:
            link.queue(ack(511, result=3, source=(2, 1)))
            link.queue(ack(511))
        elif command[2] == 510:
            link.queue(interval(1234, source=(1, 2)))
            link.queue(interval(1234, message_id=106))
            link.queue(interval(original))

    link.on_command = reply
    owner.session._close_transport()
    assert link.commands == [(1, 1, 510, 105, 0), (1, 1, 511, 105, 5000),
                             (1, 1, 511, 105, original), (1, 1, 510, 105, 0)]
    assert ('stream_interval_restored', {'requested_us': original, 'acknowledged': True,
                                         'readback_us': original}) in owner.events
    assert not any(event == 'stream_restore_failed' for event, _ in owner.events)
    assert link.closed and owner.locks[-1].closed
    assert owner.session.original_interval_us is None
    assert owner.session.interval_ack is None
    assert not owner.session.interval_requested


@pytest.mark.parametrize('original,readback', [(20_000, 5000), (-1, 5000), (-1, 0)])
def test_restore_readback_must_equal_confirmed_interval_including_disabled(owner, original, readback):
    link = open_owner(owner)
    set_original(owner, link, original)
    link.on_command = lambda command: link.queue(ack(511) if command[2] == 511 else interval(readback))
    owner.session._close_transport()
    assert any(event == 'stream_restore_failed' for event, _ in owner.events)
    assert link.closed and owner.locks[-1].closed


@pytest.mark.parametrize('missing', ['ack', 'readback'])
def test_missing_restore_evidence_is_bounded_and_releases_transport(owner, missing):
    link = open_owner(owner)
    set_original(owner, link, 20_000)

    def reply(command):
        if command[2] == 511 and missing != 'ack':
            link.queue(ack(511))
        elif command[2] == 510 and missing != 'readback':
            link.queue(interval(20_000))

    link.on_command = reply
    started = owner.clock.now_ns
    owner.session._close_transport()
    assert owner.clock.now_ns - started <= 2_100_000_000
    assert any(event == 'stream_restore_failed' for event, _ in owner.events)
    assert link.closed and owner.locks[-1].closed


@pytest.mark.parametrize('rate_changed', [False, True])
def test_armed_heartbeat_aborts_without_sending_more_interval_commands(owner, rate_changed):
    link = open_owner(owner)
    if rate_changed:
        set_original(owner, link, 20_000)
    before = list(link.commands)
    link.queue(heartbeat(armed=True))
    with pytest.raises(mavlink_imu.ArmedController):
        owner.session.poll(timeout_s=0)
    owner.session._close_transport()
    assert owner.session.armed
    assert link.commands == before
    assert link.closed and owner.locks[-1].closed


def test_arming_during_restore_stops_followup_readback_and_releases_transport(owner):
    link = open_owner(owner)
    set_original(owner, link, 20_000)
    link.on_command = lambda command: link.queue(heartbeat(armed=True))
    owner.session._close_transport()
    assert owner.session.armed
    assert link.commands[-1] == (1, 1, 511, 105, 20_000)
    assert sum(command[2] == 510 for command in link.commands) == 1
    assert link.closed and owner.locks[-1].closed


def test_disconnect_clears_pending_query_and_reconnect_needs_new_target_response(owner):
    first = open_owner(owner)
    owner.session._close_transport(restore=False)
    assert owner.session.interval_query_deadline_ns is None
    assert first.closed and owner.locks[-1].closed
    assert first.commands == [(1, 1, 510, 105, 0)]

    owner.clock.advance(3)
    owner.next_source = (42, 7)
    second = open_owner(owner)
    assert owner.session.original_interval_us is None
    assert owner.session.interval_ack is None
    assert not owner.session.interval_requested
    set_original(owner, second, 20_000)  # A reply from the previous autopilot is ignored.
    assert second.commands == [(42, 7, 510, 105, 0)]
    set_original(owner, second, 10_000, source=(42, 7))
    assert second.commands[-1] == (42, 7, 511, 105, 5000)
    assert owner.session.original_interval_us == 10_000


def test_disconnect_after_rate_change_does_not_restore_or_reuse_ack_on_reconnect(owner):
    first = open_owner(owner)
    set_original(owner, first, 20_000)
    first.queue(ack(511))
    owner.session.poll(timeout_s=0)
    assert owner.session.interval_ack == 0
    owner.session._close_transport(restore=False)
    assert len(first.commands) == 2
    assert owner.session.original_interval_us is None
    assert owner.session.interval_ack is None
    second = open_owner(owner)
    owner.clock.advance(0.3)
    owner.session.poll(timeout_s=0)
    assert second.commands == [(1, 1, 510, 105, 0)]
    assert not owner.session.interval_requested
