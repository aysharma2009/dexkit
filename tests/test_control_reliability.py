"""Fault injection for position history, asymmetric tendon control and stop coverage."""
from __future__ import annotations

import time
from unittest.mock import patch

import numpy as np
import pytest

from dexkit.config import load_hand_config
from dexkit.control.recorder import Recording, replay
from dexkit.env.dexkit_env import DexKitEnv
from dexkit.hw.feetech_hand import FeetechDriver, save_positions_state
from dexkit.hw.feetech_protocol import FeetechBus, encode_position
from dexkit.hw.mock import MockFeetechSerial, MockHand, mock_servos_for
from dexkit.hw.safety import EStop, EStopTripped, SafetyTrip, set_estop_flag
from dexkit.policy.collect import EpisodeWriter
from dexkit.util import guarded_wait


def session(cfg, start=None):
    h = MockHand(cfg, latency_s=0, bus=MockFeetechSerial(mock_servos_for(cfg, start)))
    h.connect()
    return h


def test_open_frame_survives_multiple_connections_and_power_cycles(hand_cfg):
    s = hand_cfg.servos[0]
    save_positions_state({s.id: {"pos": s.slack, "slack": s.slack}})
    for shift in (-4096, -4096, 4096, 4096, 0):
        h = session(hand_cfg, {s.id: s.slack + shift})
        assert h._slack[0] == s.slack + shift
        h.close()


def test_home_is_restored_by_a_new_session(hand_cfg):
    h = session(hand_cfg)
    s = hand_cfg.servos[0]
    new_open = s.slack + 300
    h.relax()
    h.bus_sim.servos[s.id].position = new_open
    h.rehome()
    h.close()
    h = session(hand_cfg, {s.id: new_open})
    assert h._slack[0] == new_open
    h.close()


def test_new_calibration_supersedes_old_state(hand_cfg):
    h = session(hand_cfg)
    h.close()
    s = hand_cfg.servos[0]
    s.slack += 200
    s.tight += 200
    h = session(hand_cfg)
    assert h._slack[0] == s.slack
    h.close()


def test_stall_uses_its_own_span_and_direction():
    cfg = load_hand_config()  # deliberately heterogeneous spans, including inverted servos
    s = cfg.servos[2]
    h = session(cfg, {s.id: int(s.slack + .5 * s.span)})
    h.bus_sim.servos[s.id].load_override = 900
    h.load_watch._since[2] = -1e9
    h.get_state()
    assert h._load_cap[2] == pytest.approx(.45, abs=.002)
    h.close()


def test_feedback_backoff_moves_the_goal_during_a_powered_hold(mock_hand):
    f = np.zeros(12)
    f[0] = 1
    for _ in range(20):
        mock_hand.set_targets(f, 0)
    servo = mock_hand.bus_sim.servos[mock_hand.cfg.servos[0].id]
    old_goal = servo.goal
    servo.load_override = 1000
    mock_hand.load_watch._since[0] = time.monotonic() - 1
    mock_hand.get_state()  # no subsequent operator set_targets call
    assert servo.goal < old_goal
    assert mock_hand._load_cap[0] < 1


def test_missing_active_feedback_disables_torque(mock_hand):
    del mock_hand.bus_sim.servos[mock_hand.active_ids[0]]
    with pytest.raises(SafetyTrip, match="no state"):
        mock_hand.get_state()
    assert not mock_hand.torque_on
    assert not any(s.torque_on for s in mock_hand.bus_sim.servos.values())


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_direct_nonfinite_target_never_reaches_bus(mock_hand, bad):
    f = np.zeros(12)
    f[0] = bad
    with pytest.raises(SafetyTrip, match="non-finite"):
        mock_hand.set_targets(f, 0)
    assert not mock_hand.torque_on


def test_replay_services_hand_feedback_without_observations(mock_hand, estop):
    env = DexKitEnv(mock_hand, estop=estop)
    rec = Recording(np.array([0., .15]), np.zeros((2, 16)), np.zeros((2, 16)))
    with patch.object(mock_hand, "get_state", wraps=mock_hand.get_state) as reads:
        replay(env, rec, lead_in_s=0, rate_hz=100)
        assert reads.call_count >= 2


def test_wait_checks_pending_stop_and_torque_off(mock_hand, estop):
    estop.register(hand=mock_hand)
    estop.request("test while waiting")
    with pytest.raises(EStopTripped):
        guarded_wait(30, estop, mock_hand)
    assert not mock_hand.torque_on


def test_release_before_opposite_pull_uses_measured_position(hand_cfg):
    cfg = hand_cfg
    cfg.defaults.antagonist_release_first = True
    cfg.defaults.antagonist_payout = 0
    a, b = cfg.antagonist_pairs()[0]
    cfg.servos[a].tight = cfg.servos[a].slack + 3000
    cfg.servos[b].tight = cfg.servos[b].slack - 500
    cfg.servos[b].inverted = True
    s = cfg.servos[a]
    h = session(cfg, {s.id: s.slack + 1200})
    h.bus_sim.servos[s.id].speed_tps = 0  # outgoing tendon has NOT actually released
    f = np.zeros(12)
    f[b] = 1
    for _ in range(20):
        h.set_targets(f, 0)
    assert h.bus_sim.servos[cfg.servos[b].id].goal == cfg.servos[b].slack
    h.bus_sim.servos[s.id].position = s.slack
    h._last_feedback = 0
    for _ in range(20):
        h.set_targets(f, 0)
    assert h.bus_sim.servos[cfg.servos[b].id].goal == cfg.servos[b].effective_tight
    h.close()


def test_conflicting_pair_releases_both_in_conservative_mode(hand_cfg):
    hand_cfg.defaults.antagonist_release_first = True
    hand_cfg.defaults.antagonist_payout = 0
    h = session(hand_cfg)
    a, b = hand_cfg.antagonist_pairs()[0]
    f = np.zeros(12)
    f[a] = f[b] = 1
    for _ in range(10):
        h.set_targets(f, 0)
    for i in (a, b):
        assert h.bus_sim.servos[hand_cfg.servos[i].id].goal == hand_cfg.servos[i].slack
    h.close()


def test_payout_is_calibrated_independently_for_each_tendon(hand_cfg):
    hand_cfg.defaults.antagonist_payout = 0
    a, b = hand_cfg.antagonist_pairs()[0]
    hand_cfg.servos[a].release_ticks = 70
    hand_cfg.servos[b].release_ticks = 240
    h = session(hand_cfg)
    f = np.zeros(12)
    f[a] = 1
    assert h.fingers_to_ticks(f)[b] == hand_cfg.servos[b].slack - 240
    f[a], f[b] = 0, 1
    assert h.fingers_to_ticks(f)[a] == hand_cfg.servos[a].slack - 70
    h.close()


def test_measured_extra_payout_precedes_incoming_pull(hand_cfg):
    hand_cfg.defaults.antagonist_release_first = True
    hand_cfg.defaults.antagonist_payout = 0
    a, b = hand_cfg.antagonist_pairs()[0]
    hand_cfg.servos[b].release_ticks = 240
    h = session(hand_cfg)
    outgoing = h.bus_sim.servos[hand_cfg.servos[b].id]
    outgoing.speed_tps = 0
    f = np.zeros(12)
    f[a] = 1
    for _ in range(20):
        h.set_targets(f, 0)
    assert outgoing.goal == hand_cfg.servos[b].slack - 240
    assert h.bus_sim.servos[hand_cfg.servos[a].id].goal == hand_cfg.servos[a].slack
    outgoing.position = outgoing.goal
    h._last_feedback = 0
    h.set_targets(f, 0)
    assert h.bus_sim.servos[hand_cfg.servos[a].id].goal > hand_cfg.servos[a].slack
    h.close()


def test_unreleased_opposing_tendon_times_out(hand_cfg):
    hand_cfg.defaults.antagonist_release_first = True
    a, b = hand_cfg.antagonist_pairs()[0]
    h = session(hand_cfg, {hand_cfg.servos[b].id: hand_cfg.servos[b].slack + 300})
    h._release_since[(a, b)] = time.monotonic() - 31
    f = np.zeros(12)
    f[a] = 1
    with pytest.raises(SafetyTrip, match="failed to release"):
        h.set_targets(f, 0)
    assert not h.torque_on
    h.close()


def test_per_tendon_force_cap_cannot_exceed_global_limit(hand_cfg):
    hand_cfg.servos[0].goal_torque = 175
    hand_cfg.servos[1].goal_torque = 900
    h = session(hand_cfg)
    assert h.driver._read_u16(hand_cfg.servos[0].id, "goal_torque") == 175
    assert h.driver._read_u16(hand_cfg.servos[1].id, "goal_torque") == hand_cfg.defaults.goal_torque
    h.close()


def test_episode_numbering_gap_does_not_overwrite(tmp_path):
    (tmp_path / "episode_0000.pkl").write_bytes(b"keep0")
    old = tmp_path / "episode_0002.pkl"
    old.write_bytes(b"keep2")
    w = EpisodeWriter(tmp_path, None)
    w._pos = w._act = [np.zeros(16), np.zeros(16)]
    w._imgs = [np.zeros((2, 2, 3)), np.zeros((2, 2, 3))]
    assert w.stop().name == "episode_0001.pkl"
    assert old.read_bytes() == b"keep2"


def test_mode_read_timeout_refuses_torque(hand_cfg):
    bus = MockFeetechSerial(mock_servos_for(hand_cfg))
    with patch.object(FeetechDriver, "read_mode", return_value=None):
        with pytest.raises(SafetyTrip, match="position mode"):
            session_with_bus(hand_cfg, bus)
    assert not any(s.torque_on for s in bus.servos.values())


def session_with_bus(cfg, bus):
    h = MockHand(cfg, latency_s=0, bus=bus)
    h.connect()
    return h


def test_failed_force_limit_write_refuses_torque(hand_cfg):
    bus = MockFeetechSerial(mock_servos_for(hand_cfg))
    with patch.object(FeetechDriver, "set_goal_torque", return_value=False):
        with pytest.raises(SafetyTrip, match="configuration write"):
            session_with_bus(hand_cfg, bus)
    assert not any(s.torque_on for s in bus.servos.values())


def test_error_status_is_not_successful_write(hand_cfg):
    bus = FeetechBus(MockFeetechSerial(mock_servos_for(hand_cfg)))
    with patch.object(bus, "_transact", return_value=(4, b"")):
        assert not bus.write(1, 44, [0, 1])


def test_external_stop_is_serviced_by_direct_commands(mock_hand, estop):
    DexKitEnv(mock_hand, estop=estop)
    set_estop_flag("fault injection")
    with pytest.raises(EStopTripped):
        mock_hand.set_targets(np.zeros(12), 0)
    assert not mock_hand.torque_on


@pytest.mark.parametrize("ticks", [-32768, 32768, 65535])
def test_position_encoding_refuses_sign_bit_overflow(ticks):
    with pytest.raises(ValueError, match="15-bit"):
        encode_position(ticks)


def test_gantry_close_saves_fresh_stopped_position(mock_gantry):
    from dexkit.hw.grbl_gantry import GrblGantry
    mock_gantry.set_zero()
    mock_gantry.move_to(20, 0, 0, feed=600)
    time.sleep(.02)
    with patch.object(mock_gantry, "save_frame", lambda: GrblGantry.save_frame(mock_gantry)):
        mock_gantry.close()
    saved = mock_gantry.load_saved_frame()
    assert saved["xyz"][0] > 0
    assert saved["xyz"][0] == pytest.approx(mock_gantry.sim.mpos[0], abs=.002)


def test_immediate_relax_bypasses_calibration_and_never_enables_torque(hand_cfg):
    from dexkit.tools.relax import main

    assert not hand_cfg.is_calibrated
    bus = MockFeetechSerial(mock_servos_for(hand_cfg))
    for servo in bus.servos.values():
        servo.mem[40] = 1
    set_estop_flag("already stopped")
    with (patch("dexkit.config.load_hand_config", return_value=hand_cfg),
          patch("dexkit.hw.mock.MockFeetechSerial", return_value=bus),
          patch("dexkit.hw.feetech_hand.FeetechHand.connect", side_effect=AssertionError("must not connect"))):
        main(["--mock", "--now"])
    assert not any(s.torque_on for s in bus.servos.values())
    assert len(bus.log) == 2  # broadcast + sync torque off, no startup/position/torque-on writes


def test_manual_calibration_obeys_existing_stop_flag(hand_cfg):
    from dexkit.tools.calibrate_hand import ScriptedPrompter, _step_to_tight

    bus = MockFeetechSerial(mock_servos_for(hand_cfg))
    driver = FeetechDriver(FeetechBus(bus), hand_cfg.register_map())
    set_estop_flag("stop calibration")
    with pytest.raises(EStopTripped):
        _step_to_tight(driver, hand_cfg.servos[0].id, 2048, hand_cfg,
                       ScriptedPrompter(), 0, 1, 2048, [])
    assert not any(s.torque_on for s in bus.servos.values())
    assert all(packet[5] != 42 for packet in bus.log)  # no position goal after the stop


def test_gantry_pending_stop_is_checked_before_homing(gantry_cfg):
    from dexkit.hw.mock import MockGantry

    g = MockGantry(gantry_cfg, homing=True)
    g.connect()
    e = EStop()
    e.register(gantry=g)
    g.estop = e
    e.request("stop before home")
    with pytest.raises(EStopTripped):
        g.home()
    assert "$H" not in g.sim.lines
    g.close()


def test_pose_arrival_timeout_stops_instead_of_continuing(mock_hand, estop):
    from dexkit.control.poses import Pose, go_to_pose
    from dexkit.util import Rate

    for servo in mock_hand.bus_sim.servos.values():
        servo.speed_tps = 0
    f = np.zeros(12)
    f[0] = .5
    with pytest.raises(SafetyTrip, match="not reached"):
        go_to_pose(mock_hand, Pose(f), .01, 100, estop=estop,
                   arrive_timeout_s=.01, rate=Rate(1000, sleep=lambda _: None))
    assert not mock_hand.torque_on
