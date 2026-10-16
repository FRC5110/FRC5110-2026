# Shooter Feed Tuning Runbook

Last updated: 2026-03-15

## Purpose

This runbook is for validating why the conveyor does or does not feed during shooter operation before changing code.

The goal is to distinguish between:

- intentional feed lockout
- bad command inputs
- bad distance inputs
- shooter tuning that prevents the flywheel from reaching ready-to-feed state

Timed autonomous shoot commands are out of scope for the main procedure in this document. This runbook focuses on the current active teleop and calibration workflows.

## Current Active Controls For This Workflow

- `AUX RB`: spin shooter at distance-based RPM
- `AUX LB`: feed conveyor when shooter is ready
- `AUX Back`: run distance-based shoot + feed
- `AUX Start`: spin shooter at preset RPM
- `AUX Left stick click`: run shooter distance calibration workflow
- `AUX B`: run hub-distance validation only

Important notes:

- `AUX B` does not spin the shooter or feed the conveyor. It only validates the hub-distance pipeline.
- `AUX Left stick click` is a calibration workflow. It reads the current hub distance, but the shooter target RPM comes from the calibration entry, not from automatic distance RPM.
- Distance-based shooter commands currently latch their RPM target once when the command starts. If the distance path is invalid at that moment, they can latch `0 RPM`.

## How Feed Lockout Works

The conveyor is intentionally blocked from feeding until the shooter reports `ready-to-feed`.

In plain language, the sequence is:

1. shooter spins to target
2. shooter must stay close enough to target for a short stable time
3. conveyor is allowed to feed one short packet
4. conveyor pauses briefly for recovery
5. if the shooter is ready again, another packet is allowed

This is not a circular dependency. The conveyor does not need to run before the shooter can become ready. The lockout exists to avoid firing while the flywheel is still accelerating or recovering.

## Baseline Test Order

Use this order so distance issues do not get mixed into feed-gate diagnosis.

### Safety And Setup

- Start with the robot secured and the shooting area clear.
- Start with no fuel loaded for the first dry lockout checks.
- Use a fresh battery or record battery condition for every test.
- Record every tuning change before deployment and after deployment.
- Change one parameter at a time.

### Step 1: Validate Hub Distance Only

Hold `AUX B`.

Use this step to confirm the Limelight hub-distance path is valid before any distance-based shot tests.

Pass signs:

- `AutoShoot/HubDistanceSource` is not `NoLimelightDistance`
- `AutoShoot/HubDistanceMetersSelected` is finite
- `AutoShoot/HubDistanceMetersDirectRaw` is finite
- `AutoShoot/HubDistanceDirectVisibleTagCount` is at least `1`

If this step fails, do not tune feed lockout yet. Fix distance visibility first.

### Step 2: Validate Preset Shooter Spin Only

Hold `AUX Start`.

Use this step to confirm the flywheel can reach preset target RPM without involving Limelight distance.

Watch:

- `Shooter/TargetRPM`
- `Shooter/CurrentRPM`
- `Shooter/ErrorRPM`

Pass signs:

- target RPM is nonzero
- current RPM rises smoothly toward target
- error shrinks and stays reasonably small

### Step 3: Validate Feed-Only Lockout

Hold `AUX LB` with no shooter spin command active.

Expected result:

- conveyor should remain locked out
- this is intentional

This confirms the system does not feed just because feed is requested.

### Step 4: Validate Preset Shot Feed

Hold `AUX Start`, then hold `AUX LB`.

This is the cleanest feed-gate test because it removes distance as a variable.

Expected healthy behavior:

- shooter reaches preset target RPM
- conveyor stays blocked until the shooter is ready
- conveyor begins short feed packets
- brief pauses between packets are visible

If this step fails, the problem is not the Limelight distance path.

### Step 5: Validate Distance-Based Spin

Hold `AUX RB`.

Use this step to confirm the distance-based shot command latched a valid RPM.

Pass signs:

- `Shooter/TargetRPM` is nonzero
- `AutoShoot/HubDistanceSource` indicates a valid source
- shooter behaves similarly to the preset spin test

If `Shooter/TargetRPM` is `0`, the distance-based command likely latched an invalid distance.

### Step 6: Validate Distance-Based Combined Shoot + Feed

Hold `AUX Back`.

Expected healthy behavior:

- a valid nonzero shooter target is latched at command start
- shooter reaches the ready band
- conveyor begins packet feeding

If this fails after Step 4 passed, treat it as a distance-input or command-latching problem before touching feed-gate tuning.

### Step 7: Use Calibration Mode Only If More Detail Is Needed

Hold `AUX Left stick click`.

Use this step when you need deeper telemetry about:

- latched distance
- suggested RPM
- current target RPM
- whether the shooter is actually reaching `ready-to-feed`
- whether auto feed is enabled or force feed is active

## Lockout Scenarios To Reproduce

These scenarios are useful because each points at a different cause.

### 1. No Shooter Command Active

Reproduce:

- hold `AUX LB` by itself

Expected result:

- conveyor does not feed

Interpretation:

- intentional lockout
- not a bug

### 2. Distance-Based Command Latched `0 RPM`

Reproduce:

- point the robot so hub distance is invalid
- start `AUX RB` or `AUX Back`

Expected result:

- `Shooter/TargetRPM` stays at `0`
- conveyor never feeds

Interpretation:

- bad command input caused by invalid distance at command start
- not a feed-gate tuning problem

### 3. Shooter Has Nonzero Target But Never Reaches Ready Band

Reproduce:

- run `AUX Start` plus `AUX LB`
- or run `AUX Back` with valid distance

Expected result:

- target RPM is nonzero
- current RPM never gets close enough or stable enough to feed

Interpretation:

- likely shooter-performance or readiness-threshold issue

### 4. Packet Recovery Mistaken For A Failure

Reproduce:

- run a healthy preset shot test with fuel loaded

Expected result:

- conveyor feeds
- conveyor pauses briefly
- conveyor feeds again

Interpretation:

- this short pause is intentional packet recovery
- do not tune it out unless the recovery pause is clearly too long

### 5. Shooter Hits Speed Briefly But Does Not Stay Ready Long Enough

Reproduce:

- run preset shot and watch current RPM oscillate around target

Expected result:

- current RPM gets close to target
- feed still starts late or never starts

Interpretation:

- stable-time or filtering may be too strict
- or the flywheel is too unstable under load

### 6. Future Boost Hold Scenario

Current code has boost disabled by default, but if boost is enabled later, `PAUSE_FEED_WHEN_BOOST_DEFAULT` can intentionally hold feed until boost ends.

Interpretation:

- future-use diagnostic
- not a primary current failure mode

## Dashboard Signals To Watch

Use these keys because they exist in the current code.

### Core Shooter Telemetry

- `Shooter/TargetRPM`
- `Shooter/CurrentRPM`
- `Shooter/ErrorRPM`

### Distance Telemetry

- `AutoShoot/HubDistanceMetersSelected`
- `AutoShoot/HubDistanceSource`
- `AutoShoot/HubDistanceMetersDirectRaw`
- `AutoShoot/HubDistanceDirectVisibleTagCount`

### Shooter Calibration Telemetry

- `AutoShoot/ShooterCalibrationState`
- `AutoShoot/ShooterCalibrationLatchedDistanceMeters`
- `AutoShoot/ShooterCalibrationLatchedDistanceSource`
- `AutoShoot/ShooterCalibrationSuggestedRPM`
- `AutoShoot/ShooterCalibrationCurrentTargetRPM`
- `AutoShoot/ShooterCalibrationCurrentRPM`
- `AutoShoot/ShooterCalibrationAtSpeed`
- `AutoShoot/ShooterCalibrationCanFeed`
- `AutoShoot/ShooterCalibrationFeedEnabled`
- `AutoShoot/ShooterCalibrationForceFeed`
- `AutoShoot/ShooterCalibrationReadyToFeed`
- `AutoShoot/ShooterCalibrationUsingBoost`

Important note:

- `Shooter/FeedState` and `AutoShoot/State` are set at command start and end, but many of the continuous updates are currently disabled in code. Do not rely on those strings as the primary diagnosis tool.

## What To Tune, When, And How

Use the preset-RPM tests before changing anything in the distance-based path.

| Parameter | What It Controls | When To Change It | How To Change It | Risk If Too Far |
| --- | --- | --- | --- | --- |
| `FEED_READY_TOL_RPM_DEFAULT` (`175 RPM`) | How close the shooter must be to target before feed is allowed | Shooter is clearly near target but never enters feed | Loosen in small steps, about `+25` to `+50 RPM` at a time | Too loose allows weak shots and early feed |
| `FEED_READY_STABLE_SEC_DEFAULT` (`0.08 s`) | How long RPM must stay ready before feed starts | Shooter reaches target but feed starts too late or never starts because readiness resets | Reduce gradually, about `0.01` to `0.02 s` at a time | Too small allows feed during transient overshoot |
| `FEED_RPM_FILTER_ALPHA_DEFAULT` (`0.30`) | Smoothing on the RPM used for feed readiness | Raw RPM behavior looks acceptable but ready-to-feed reacts too slowly or chatters | Increase alpha if readiness reacts too slowly, decrease alpha if readiness chatters | Too much smoothing delays feed, too little makes readiness noisy |
| `CONVEYOR_FEED_POWER_DEFAULT` (`0.60`) | Feed power during packet commit | Feed readiness is correct, but the conveyor under-feeds, stalls, or hits too hard | Change only after readiness behavior is confirmed; adjust by small steps such as `0.05` | Can hide the real problem by causing RPM collapse or weak feed |
| `FEED_PACKET_SEC_DEFAULT` (`0.22 s`) | How long each feed packet lasts | The system starts feeding, but packets are too short to move fuel or too long and cause big RPM sag | Adjust in small steps, about `0.02 s` at a time | Long packets can look like poor recovery instead of bad packet length |
| `FEED_RECOVERY_MIN_SEC_DEFAULT` (`0.06 s`) | Forced pause between packets | The system feeds once but waits too long to resume, or re-enters too aggressively | Reduce if recovery is clearly too long; increase if repeated packets collapse RPM | Too short hammers the flywheel, too long feels like lockout |
| `PAUSE_FEED_WHEN_BOOST_DEFAULT` (`true`) | Whether boost mode blocks feed | Only relevant if boost is enabled later | Leave alone for current testing; revisit only if boost is intentionally turned on | Can create confusing feed delays if changed without understanding boost behavior |
| `PID_P_DEFAULT` (`0.006`) | Shooter velocity-loop aggressiveness | Flywheel cannot settle into the ready band even in preset-RPM tests | Adjust only after confirming feed thresholds are not the real issue; move in small steps | Too high can add oscillation and make readiness worse |
| `PID_D_DEFAULT` (`0.0`) | Damping in the shooter velocity loop | Only if closed-loop response becomes oscillatory after other shooter tuning | Add or reduce in very small steps | Too much D slows response and can block readiness |
| `CURRENT_LIMIT_AMPS_DEFAULT` (`60`) | Electrical limit on shooter acceleration | Flywheel consistently accelerates too slowly and never gets into the ready band soon enough | Raise cautiously only if hardware and electrical margin allow it | Too high increases current stress and battery sag |
| `FEEDFORWARD_KV_VOLTS_PER_RPS_DEFAULT`, `FEEDFORWARD_KS_VOLTS_DEFAULT`, `FEEDFORWARD_KA_VOLTS_PER_RPS2_DEFAULT` | Flywheel feedforward terms | Only after simpler readiness tuning shows the flywheel itself is the limiting factor | Treat as secondary guidance; use after verifying base spin-up and recovery behavior | Bad feedforward values can distort the whole shooter response |

Important rule:

- `PID_P_DEFAULT`, `PID_D_DEFAULT`, `CURRENT_LIMIT_AMPS_DEFAULT`, and feedforward constants are shooter-performance parameters, not feed-gate parameters. Touch them only if the flywheel cannot reliably reach the ready band.
- In the current runtime path, `kV` is the feedforward term that actively matters most. `kS` and `kA` are still worth recording, but they are secondary for immediate on-robot feed diagnosis.

## Tuning Rules

- Do not tune conveyor power first if the shooter never gets ready.
- Do not loosen readiness thresholds until preset-RPM spin tests show the flywheel is physically close to target.
- Use preset-RPM tests before distance-based tests so Limelight issues do not contaminate feed-gate diagnosis.
- Change one parameter at a time.
- Record old value, new value, battery state, test scenario, and result after each change.
- Re-test the same scenario after each change before moving on.
- If preset-RPM feed works and distance-based feed does not, stop tuning the feed gate and debug distance input instead.

## Quick Diagnosis Guide

- `TargetRPM = 0`
  - likely bad command input or bad latched distance
  - do not tune feed thresholds first

- `TargetRPM > 0`, `CurrentRPM` never gets close
  - likely shooter-performance problem
  - inspect shooter tuning before feed-gate tuning

- `CurrentRPM` gets close, but feed never starts
  - inspect `FEED_READY_TOL_RPM_DEFAULT`, `FEED_READY_STABLE_SEC_DEFAULT`, and `FEED_RPM_FILTER_ALPHA_DEFAULT`

- Feed starts, then pauses
  - this may be normal packet recovery
  - inspect `FEED_PACKET_SEC_DEFAULT` and `FEED_RECOVERY_MIN_SEC_DEFAULT` only if the behavior is clearly too weak or too slow

- Preset shot works, distance-based shot does not
  - likely distance validation or command-latch problem
  - do not tune shooter readiness first

## Pass / Fail Criteria

Pass means all of the following are true:

- preset spin plus manual feed produces repeatable packet feeding
- distance-based combined shot feeds when valid distance is present
- intentional lockout scenarios are distinguishable from real failures by telemetry
- the operator can tell whether a failure is:
  - no target or bad command input
  - flywheel not reaching ready state
  - gate intentionally in recovery
  - distance path invalid at command start

Fail means any of the following are true:

- preset shot cannot produce repeatable feed packets
- distance-based shot feeds inconsistently because target RPM latches invalid values
- operators cannot tell the difference between intentional recovery and a blocked shot
- tuning changes are being made before the failing scenario is clearly identified

## Recommended Test Notes Template

For every test, record:

- date and battery condition
- scenario run
- fuel loaded or dry test
- target RPM
- observed current RPM behavior
- whether feed started
- whether pauses looked intentional
- parameter changed, if any
- result after the change
