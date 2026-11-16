# Shooter Guide

Last verified against `branch-testing`: 2026-11-16. This guide describes the checked-in implementation; verify hardware, dashboard setup, and current constants before testing or tuning. Code is the source of truth for behavior and defaults.

## Scope and source files

Shooter behavior is primarily implemented in:

- `src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java` — flywheel/feed-wheel control, readiness, telemetry, and SysId routine.
- `src/main/java/frc/robot/commands/ShooterFeedGate.java` — packet-feed/recovery state machine.
- `src/main/java/frc/robot/commands/SpinShooterAtRpmCommand.java`, `AutoGetDistanceShootFeedCommand.java`, `FeedConveyorWhenShooterReadyCommand.java`, and `CalibrateShooterDistanceCommand.java` — active shooter command workflows.
- `src/main/java/frc/robot/RobotContainer.java` — controller bindings, hub-distance calculation, dashboard tuning/calibration access.
- `src/main/java/frc/robot/HubTagVision.java` — Limelight hub-tag estimate.
- `src/main/java/frc/robot/Constants.java` — hardware IDs and default tuning values.

## Hardware and control overview

The shooter assembly currently uses:

- Flywheel leader TalonFX: CAN ID 32.
- Flywheel follower TalonFX: CAN ID 33.
- Shooter feed-wheel Spark Flex: CAN ID 19.

The flywheel uses CTRE Phoenix 6 velocity control, requesting torque-current FOC when available and falling back to velocity-voltage control if needed. The follower follows the leader. The separate Spark Flex feed wheel uses its own velocity controller and is commanded to its configured shooting RPM when the flywheel has a nonzero target. The conveyor that feeds game pieces is a separate subsystem and is governed by shooter readiness in the feed commands.

The flywheel SysId routine temporarily commands voltage to the leader, keeps the follower following, and logs applied voltage, flywheel position, and velocity. The routine is implemented in `ShooterFlywheelSubsystem`; it does not use the old Spark Flex flywheel-control path described in earlier notes.

## Active operator controls

Controller indices and bindings are defined in `RobotContainer.java`:

| Controller | Control | Action |
| --- | --- | --- |
| AUX | Right bumper | Spin at Limelight distance-based RPM |
| AUX | Left bumper | Feed conveyor when shooter is ready; requires a separate active shooter command |
| AUX | Back | Distance-based shoot and feed |
| AUX | Start | Spin at preset RPM |
| AUX | Left-stick click | Shooter-distance calibration; only while Pit tuning controls are enabled |
| AUX | Right-stick click | Force-feed input while shooter-distance calibration is running |
| AUX | B | Hub-distance validation; only while Pit tuning controls are enabled |
| Driver, Test mode | B | SysId quasistatic forward |
| Driver, Test mode | Left bumper | SysId quasistatic reverse |
| Driver, Test mode | Right bumper | SysId dynamic forward |
| Driver, Test mode | Start | SysId dynamic reverse |

Do not treat calibration controls as generally available in a match. `RobotContainer` enables Pit/Debug profiles only when FMS is not attached and the robot is disabled or in Test; shooter calibration additionally requires Pit profile with tuning armed. Confirm the dashboard’s active profile/tuning status before relying on those controls.

## Distance-based shooting

The distance-to-shot-RPM path estimates the preferred fuel-hub center from Limelight hub tags using `HubTagVision`. `RobotContainer` uses the estimate’s robot-space distance and briefly retains the last valid tag distance (up to 0.10 seconds) to bridge short gaps. If a valid measurement or recent latched measurement is unavailable, the selected distance is invalid and the distance RPM supplier returns 0; the selected shooting-distance path does not substitute odometry distance.

The shooter maps distance to RPM by interpolating the table in `Constants.ShooterTune.DISTANCE_POINT_METERS_DEFAULTS` and `RPM_POINT_DEFAULTS`. A distance RPM trim is then applied. Current table values and trim default are in `Constants.java`; update the constants from measured test data rather than copying old values from session notes.

Distance-based spin and combined shoot/feed commands refresh their target from the RPM supplier. A finite, nonzero candidate replaces the command’s stored target; an invalid candidate does not replace the last valid target. If the command starts without a valid target, it can remain at zero until it receives a valid candidate.

Useful distance dashboard keys include:

- `AutoShoot/HubDistanceSource`
- `AutoShoot/HubDistanceMetersSelected`
- `AutoShoot/RPMFromDistance`
- `AutoShoot/LimelightTagCount`
- `AutoShoot/LimelightAvgTagDistMeters`

Additional raw estimate and latch diagnostics are published in Pit/Debug telemetry; their availability is profile-dependent.

## Shooter readiness and feed behavior

The flywheel’s `atSpeed` status and `ready-to-feed` status are distinct. Feed readiness uses filtered flywheel RPM, the configured readiness tolerance and stable interval, and boost pause behavior if boost is enabled. When the feed wheel is commanded, the subsystem also requires it to be healthy before reporting ready-to-feed.

`ShooterFeedGate` feeds in packets:

1. Wait until the shooter reports ready-to-feed.
2. Feed for `FEED_PACKET_SEC_DEFAULT` (currently 0.22 s).
3. Stop feeding for at least `FEED_RECOVERY_MIN_SEC_DEFAULT` (currently 0.06 s).
4. Wait for readiness again before another packet.

`FeedConveyorWhenShooterReadyCommand` only requests this gate; it does not spin up the shooter. Thus AUX left bumper alone should remain blocked unless another command is running the shooter to a valid target. The combined distance command handles both spinning and gated feeding. The calibration command has separate dashboard-controlled auto-feed and right-stick force-feed inputs.

## Recommended diagnosis sequence

Use a clear, secured test area, keep people clear of moving mechanisms, and begin dry tests without game pieces. Record battery condition and change one parameter at a time.

1. **Validate distance first.** In an eligible Pit tuning setup, use AUX B and check `AutoShoot/HubDistanceSource`, `AutoShoot/HubDistanceMetersSelected`, `AutoShoot/HubDistanceMetersDirectRaw`, and `AutoShoot/HubDistanceDirectVisibleTagCount`. A missing/invalid distance is a vision or setup issue, not a feed-gate tuning issue.
2. **Test preset spin.** Hold AUX Start. Check `Shooter/TargetRPM`, `Shooter/CurrentRPM`, and `Shooter/ErrorRPM`. This isolates the flywheel from distance estimation.
3. **Check feed lockout.** Hold AUX left bumper without a shooter command. The conveyor should not feed because the shooter is not ready.
4. **Test preset spin plus feed.** Hold AUX Start and AUX left bumper. Confirm the shooter reaches its ready band and the conveyor feeds in packets with recovery pauses.
5. **Test distance-based spin.** Hold AUX right bumper. Confirm a valid distance source and nonzero target RPM. If the target remains zero, first investigate whether the distance estimate is valid.
6. **Test combined distance shoot/feed.** Hold AUX Back. If preset spin/feed works but this does not, focus on the distance input or command target rather than changing feed thresholds first.
7. **Use calibration only when needed.** In eligible Pit tuning mode, AUX left-stick click runs calibration. It latches a distance when available, displays a suggested RPM, and uses the dashboard calibration target. Auto-feed is separately enabled on the dashboard; AUX right-stick click is the force-feed input. Use force feed only with appropriate safety precautions.

### Useful shooter telemetry

Regular shooter telemetry includes:

- `Shooter/TargetRPM`, `Shooter/CurrentRPM`, `Shooter/ErrorRPM`
- `Shooter/FlywheelAtSpeed`
- `Shooter/FeedWheelTargetRPM`, `Shooter/FeedWheelCurrentRPM`, `Shooter/FeedWheelHealthy`, `Shooter/FeedWheelFault`
- `Shooter/ReadyToFeed`, `Shooter/ReadyToFeedSource`, `Shooter/UsingBoost`
- Hardware availability flags for the flywheel leader, follower, and feed wheel

Pit/Debug mode adds detailed flywheel position, applied voltage, torque current, control mode, and hardware-fault telemetry. Calibration exposes `AutoShoot/ShooterCalibration...` values including latched distance/source, suggested/current target RPM, current RPM, readiness, feed-enabled, and force-feed state. Some `Shooter/FeedState` and `AutoShoot/State` strings are set at command boundaries and are not continuous progress indicators; prefer the live RPM/readiness fields when diagnosing.

## Tuning reference

The following are defaults in `Constants.ShooterTune`; verify them there before a test because this guide intentionally avoids duplicating every adjustable value.

- **Distance mapping:** `DISTANCE_POINT_METERS_DEFAULTS`, `RPM_POINT_DEFAULTS`, and `DISTANCE_RPM_TRIM_DEFAULT`.
- **Readiness/feed:** `FEED_READY_TOL_RPM_DEFAULT` (175 RPM), `FEED_READY_STABLE_SEC_DEFAULT` (0.08 s), `FEED_RPM_FILTER_ALPHA_DEFAULT` (0.30), `FEED_PACKET_SEC_DEFAULT` (0.22 s), `FEED_RECOVERY_MIN_SEC_DEFAULT` (0.06 s), `FEED_DROPOUT_TOL_RPM_DEFAULT`, `CONVEYOR_FEED_POWER_DEFAULT`, and `CONVEYOR_HOLD_POWER_DEFAULT`.
- **Flywheel:** `PID_P_DEFAULT` (0.004), `PID_I_DEFAULT` (0.0), `PID_D_DEFAULT` (0.0), plus the CTRE slot, current-limit, inversion, and motor-ratio constants. The CTRE slot gains are separate constants from the older REV shooter gains.
- **Feed wheel:** `FEED_WHEEL_*` constants, including the Spark Flex PID/feedforward, current limit, target RPM, and health thresholds.
- **SysId:** `SYSID_RAMP_RATE_VOLTS_PER_SEC_DEFAULT` (1.0 V/s), `SYSID_STEP_VOLTAGE_DEFAULT` (7.0 V), and `SYSID_TIMEOUT_SEC_DEFAULT` (10.0 s).

Tuning dashboard controls currently apply the shooter distance RPM trim through the Pit tuning workflow. Do not assume dashboard PID/gain entries are live controls: the motor gains and most behavior values are compile-time constants in the checked-in implementation. Tune the distance table from recorded shots; diagnose distance, readiness, flywheel performance, and feed-wheel health separately before adjusting conveyor power.

The `FEEDFORWARD_KS_VOLTS_DEFAULT`, `FEEDFORWARD_KV_VOLTS_PER_RPS_DEFAULT`, and `FEEDFORWARD_KA_VOLTS_PER_RPS2_DEFAULT` values in `Constants.java` are documented as SysId-style values, but the current flywheel control path does not apply those constants. The active CTRE flywheel gains are `FLYWHEEL_SLOT0_*`; the feed wheel has its own Spark Flex velocity feedforward. Do not assume changing the generic `FEEDFORWARD_*` constants changes flywheel runtime behavior.

## Shooter SysId procedure

### Before running

- Use Test mode only; verify the robot is securely positioned and the shooter/conveyor path is empty.
- Keep people and loose objects clear of the shooter. Be prepared to disable the robot if behavior is unexpected.
- Deploy the code being tested and confirm the current SysId settings in `Constants.java`.
- Verify how the test data will be recorded. `Constants.Logging.WPILIB_DATA_LOGGING_ENABLED` is currently `false` by default, so do not assume a `.wpilog` is being saved without checking logging configuration.

### Run the tests

Run one direction at a time, holding the corresponding driver button for each routine and releasing to cancel:

1. B — quasistatic forward
2. Left bumper — quasistatic reverse
3. Right bumper — dynamic forward
4. Start — dynamic reverse

The SysId routine commands direct voltage to the flywheel leader and logs voltage, position, and velocity. Open the recorded log with WPILib’s desktop SysId tool and inspect the characterization results. If the expected data is missing, resolve logging configuration before repeating tests.

### Interpreting and applying results

SysId estimates `kS`, `kV`, and `kA`. The subsystem logs angular velocity in rotations per second. Keep units consistent when interpreting results. Before changing constants, identify which runtime controller consumes each value:

- Current CTRE flywheel control uses the `FLYWHEEL_SLOT0_*` slot constants; these currently default to zero.
- The generic `FEEDFORWARD_*` constants are not consumed by the current flywheel control path.
- The Spark Flex feed wheel uses `FEED_WHEEL_PID_FF_DEFAULT` in its RPM-based controller.

Do not copy SysId values into an unrelated constant. If implementing a new feedforward path, update the code and its tests/documentation together. After a tuning change, rebuild, deploy, verify startup and motor response with the robot secured, then validate unloaded and under controlled shot load.

## Test record template

For each test, record:

- Date, code revision, and battery condition
- Test mode/command and whether game pieces were loaded
- Distance source and selected distance, for distance-based tests
- Shooter target/current RPM and feed-wheel health
- Whether readiness was reached and feed packets occurred
- Constant changed (old/new value), reason, and observed result
- SysId routine/log filename and analyzed values, if applicable
