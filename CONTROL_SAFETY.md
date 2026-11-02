# Controlling an imprecise tendon hand

The servos control **spool position**. They do not measure finger angle, cable tension,
or fingertip force. Fabric stretch, friction, slack, spool radius and routing make
forward and backward motion different. Equal rotations must not be treated as equal
finger movement or equal force. A recorded pose is an empirical cable setting.

## Independent calibration

Each tendon has its own `slack`, `tight`, winding direction (`inverted`), step limit,
stall threshold, optional `goal_torque`, and optional `release_ticks` in `hand.yaml`.
The flexor and extensor need separate measurements. Do not copy either endpoint,
travel distance, or torque cap from its opposing tendon.

- `slack`: the measured neutral/released spool position.
- `tight`: the measured maximum useful pull for this tendon at controlled torque.
- `release_ticks`: optional measured extra unwind **past this tendon's slack** that
  permits the opposite tendon to move. This is a positive distance in this servo's
  own ticks; its winding direction determines the sign. Start at zero and only
  increase after observing that specific cable. Excessive unwind can lose the cable
  from its spool. The global pay-out cap still applies.
- `goal_torque`: optional individual servo torque setting. It can lower, but cannot
  raise, the global operating cap. A value such as 200 is a servo register value,
  not 200 newtons or a calibrated cable force.

For example, **illustrative values only**, one tendon might need 70 ticks of extra
release and a cap of 175, while the other needs 240 ticks and a cap of 250. No
relationship between those values is assumed. Use measurements from your hardware.

The old `defaults.antagonist_payout` ratio remains for explicit legacy experiments.
Its default is now zero. Prefer `release_ticks` over deriving the release of one
servo from the rotation of the other.

## Direction reversal

`defaults.antagonist_release_first: true` is the conservative default. For a pair:

1. Command the outgoing tendon toward its independently calibrated release target.
2. Read its actual servo position. Advancing its goal alone is not enough.
3. Begin pulling the incoming tendon after that release target is reached within
   tolerance (no more than 20 ticks, further restricted by the normalized tolerance).
4. If release fails within `antagonist_release_timeout_s` (default 30 seconds),
   disable torque and raise a safety error.

When both opposing pulls exceed the configured release tolerance, the pair is sent
toward neutral instead of deliberately tensioning both sides. Intermediate pose
waypoints that overlap directions therefore release through neutral. The legacy
paired scaling behavior is available only by setting `antagonist_release_first: false`.

Reaching a release spool position does **not** prove the cable is slack or the finger
has moved. Calibrate and observe the actual mechanism. Torque/load monitoring limits
exposure but cannot replace cable-tension or finger-position sensing.

## What the software monitors

Every normal hand command services feedback at least once per `feedback_period_s`
(default 0.1 seconds, subject to serial/loop latency). Replay no longer bypasses this
monitoring. Powered sequence waits, one-shot holds, operator input waits, and normal
session gantry waits also service it. Voltage/temperature checks use the configured
voltage-check period; stall duration uses the per-tendon threshold.

A missing active-servo response stops hand torque. Missing mode readings or failed
startup configuration writes block torque enable. Nonfinite targets are rejected.
The stall calculation uses the affected tendon's own span and direction. Arrival
timeout stops torque and fails the pose rather than declaring success and continuing.

SPACE requests a stop on the next control-thread check. Replay timing waits and GRBL
command/homing waits now process stop checks. The shared stop flag is checked by
manual calibration, unwind, and the ROS publisher timer too. Windows console SPACE
handling uses `msvcrt` rather than unavailable POSIX terminal functions.

`dexkit-relax --now` sends torque-off directly. It does not require completed
calibration and does not enable torque first. Use a physical power disconnect for
an independent stop if the computer, USB link or controller stops responding.

## Position history and reset

Servo power cycles can lose whole spool turns. The controller now carries the saved
live open position through subsequent sessions instead of reapplying a shift to the
original YAML on each connection. State includes a calibration signature, so a new
calibration can supersede old memory. Interactive `home` persists the newly established
open positions. Ambiguous/no-memory starts are still reported for operator confirmation;
the software cannot infer a lost multi-turn reference from a single angle reading.

`home` defines where the fingers are **now** as their released position after you
physically open them with torque off. It is not an automatic search for slack. The
wrist still uses its calibrated center on the nearest equivalent turn; `home` does
not recalibrate that center. Use wrist calibration if its neutral is wrong.

Gantry frame persistence stops motion, reads fresh stopped coordinates, and saves
those coordinates. A saved zero is invalid if the mechanism moved while unpowered.

## Bringing up opposing tendons

The shipped configuration continues to disable all five extensor channels. These
changes do not invent measurements for them or turn them on automatically.

1. With a physical stop available, identify the actual cable routing and winding
   direction for each channel. Confirm individual motion at low speed and torque.
2. Calibrate each tendon with its opposite relaxed. Manual/tight calibration now
   turns off opposing finger torques so they do not brake the measurement.
3. Establish a reproducible neutral/slack reference. Check both endpoints repeatedly
   while watching the fabric, cable and spool. Set separate conservative torque caps.
4. If the opposite side still brakes movement at neutral, measure that tendon's extra
   release distance. Do not estimate it from the other servo's rotation.
5. Enable and test **one pair** at a time. Verify release before pull in both directions,
   gentle intermediate positions, stopping, and recovery after a power cycle.
6. Tune named poses by eye and repeat after the fabric/cables settle. Keep margins
   below physical limits. Only then try choreography; a learned policy comes later.

For repeatable finger angles or contact forces across changing loads, add feedback:
camera/finger-angle sensing for motion, and cable-tension sensing for force. Software
cannot reconstruct those missing measurements reliably from servo rotation alone.

## Reproducible software checks

From an environment with the package installed:

```sh
python -m pip install -e ".[dev]"
python -m pytest
python -m ruff check src tests scripts/check_mock.py
python scripts/check_mock.py
```

The cross-platform integration script copies config and uses temporary state, and
only simulated transports. Standalone `--mock` CLI runs default to a separate
`data/mock-runtime` state directory unless `DEXKIT_DATA` is explicitly provided.

Optional policy checks require `.[policy,dev]` and `DEXKIT_POLICY_TESTS=1`. Synthetic
policy tests check the software pipeline, not grasp quality or physical control.
