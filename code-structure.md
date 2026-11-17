# Code Structure - 2026swervecode1

## Top-Level Layout
- `build.gradle`: GradleRIO build, deploy, and dependency configuration.
- `src/main/java/frc/robot`: robot lifecycle, constants, field geometry, vision helpers, and telemetry.
- `src/main/java/frc/robot/generated`: CTRE Tuner X-generated swerve hardware constants and module configuration.
- `src/main/java/frc/robot/subsystems`: drivetrain and mechanism subsystem implementations.
- `src/main/java/frc/robot/commands`: teleop, tuning, alignment, and shooting commands.
- `src/main/java/frc/robot/commands/auton`: PathPlanner autonomous event commands.
- `src/main/deploy`: PathPlanner paths/autos and other deploy-time assets.
- `vendordeps`: CTRE Phoenix 6, REVLib, PathPlanner, PhotonVision, and Grapple dependency metadata.

## Java Package Map
- `frc.robot.Main`: WPILib entrypoint.
- `frc.robot.Robot`: lifecycle callbacks, scheduler execution, telemetry updates, LED state, and mode setup.
- `frc.robot.RobotContainer`: constructs subsystems, configures controller bindings and PathPlanner autos, and supplies shooting-distance calculations.
- `frc.robot.Constants`: CAN/DIO/PWM assignments, subsystem configuration, drive limits, shooter tuning values, and alignment timeouts.
- `frc.robot.FieldGeometry` / `frc.robot.HubTagVision`: field target poses and Limelight-based hub-tag geometry/distance estimates.
- `frc.robot.LimelightHelpers`: Limelight NetworkTables/JSON helpers and pose-estimate parsing.
- `frc.robot.AllianceUtil`: alliance-aware field direction and path-flipping helpers.
- `frc.robot.Telemetry`: publishes drivetrain telemetry using the selected match, pit, or debug profile.
- `frc.robot.generated.TunerConstants`: generated CTRE swerve module constants, hardware IDs, and drive/steer gains.

## Runtime Control Flow
1. `Main` starts `Robot`.
2. `Robot.robotInit()` creates `RobotContainer`, initializes LEDs and Limelight port forwarding, and leaves vision odometry disabled.
3. `RobotContainer` constructs the drivetrain and mechanisms, configures commands/default commands, and sets up the PathPlanner autonomous chooser.
4. `Robot.robotPeriodic()` refreshes alliance state, runs `CommandScheduler`, and updates telemetry.
5. Each subsystem's `periodic()` performs its regular sensor/control work. `CommandSwerveDrivetrain.periodic()` handles drivetrain perspective/simulation and vision integration when vision odometry is enabled.
6. Vision odometry is explicitly disabled in disabled, autonomous, and teleop initialization in the current `Robot.java`; do not assume Limelight poses are currently fused into drivetrain odometry during normal operation. Hub-tag vision is also used separately for alignment and shooter distance.

## Commands
Commands own behavior and require the subsystems they control. `RobotContainer` contains the live bindings; some command classes or PathPlanner names may exist for compatibility or testing without a live driver binding.

### Alignment and driving
- `AlignToFuelHubCommand`: uses Limelight hub-tag pose estimates to calculate a target heading, applies a bounded heading PID, briefly latches a recent valid heading if vision drops, and ends after stable alignment or target-acquisition/loss timeout. It does not drive translation.
- `AutoAlignFieldCentric`: uses the drivetrain pose and a selected field target (fuel hub or outpost) to command X/Y/heading motion with profiled PID controllers; finishes when the target is stable or unavailable.
- `RobotContainer` default drivetrain command: field-centric manual translation and rotation from the driver left stick/right X. Driver POV up/down invoke heading-hold commands; driver A/Y reset field orientation (A attempts vision heading first, Y uses gyro seeding).

### Intake, conveyor arm, and conveyor
- `ManualConveyorArmCommand`: scales and deadbands the auxiliary right-stick input to move the arm manually; stopping the command holds the current position when possible.
- `MoveConveyorArmToAbsolutePositionCommand`: moves the arm to an absolute-encoder endpoint/preset and ends on target, timeout, or safety failure.
- `RunIntakeRollerAtRpmCommand`: runs the intake roller at a requested RPM (the live bindings use forward/reverse fixed setpoints).
- `RunConveyorAtRpmCommand` / `RunConveyorPercentCommand`: run the conveyor using velocity control or direct percent output.
- `RunIntakeForwardAutoCommand`, `RunIntakeReverseAutoCommand`, `RunConveyorForwardAutoCommand`, and `RunConveyorReverseAutoCommand`: simple autonomous/event-marker mechanism actions, typically wrapped with a timeout.

### Shooter and feed
- `SpinShooterAtRpmCommand`: latches a valid requested RPM, spins the flywheel and its associated feed wheel, updates optional hub-tag LED feedback, and stops the shooter when interrupted/ended. Used for preset or distance-based spin-up.
- `FeedConveyorWhenShooterReadyCommand`: runs the conveyor only when the shooter feed gate reports ready; otherwise applies the configured hold output. It does not set the flywheel target itself.
- `ShootFromDistanceCommand`: legacy/general combined shooter/conveyor command. In the current implementation it uses the preset shot RPM; its distance supplier is not used to select RPM. Check before treating the class name as distance-based behavior.
- `AutoGetDistanceShootFeedCommand`: obtains RPM from its supplier (currently distance-to-RPM in the primary binding), latches the last valid target, runs the shooter, and gates conveyor feed on readiness.
- `ShootAndFeedWhenReady`, `AutoShootFeedWhenReadyCommand`, and `TimedShootAndFeedWhenReadyCommand`: coordinated spin-up/feed variants used by bindings or autonomous helpers; timed autonomous variants end after their configured duration.
- `ShooterFeedGate`: shared feed state machine: wait until ready, feed a short packet, pause for recovery, and require readiness again before another packet.
- `CalibrateShooterDistanceCommand` and `CalibrateHubDistanceCommand`: pit/debug workflows for observing/calibrating shooter distance/RPM or hub-distance telemetry. Tuning actions are gated by pit/test eligibility in `RobotContainer`.
- Shooter SysId commands are exposed by `ShooterFlywheelSubsystem`; driver B/LB/RB/Start run quasistatic forward/reverse and dynamic forward/reverse tests in test mode.

## Subsystems and Hardware
CAN IDs below are the current assignments from `Constants.CanIds` and `TunerConstants`; update this list when wiring/constants change.

- `CommandSwerveDrivetrain` (CTRE Phoenix 6 swerve): field-centric drive requests, pose/heading access, simulation, alliance perspective, and optional Limelight pose fusion.
  - Pigeon2: CAN 13
  - Front Left: drive 5, steer 4, encoder 6
  - Front Right: drive 8, steer 7, encoder 9
  - Back Left: drive 2, steer 1, encoder 3
  - Back Right: drive 11, steer 10, encoder 12
- `ConveyorArmSubsystem` (two TalonFX motors and DIO absolute encoder): manual movement, encoder-limit protection, endpoint positioning, and PID/gravity-compensated position hold. Leader CAN 20, follower CAN 21, absolute encoder DIO 0. `IntakeArmSubsystem` is a deprecated compatibility subclass; the active container constructs `ConveyorArmSubsystem`.
- `IntakeRollerSubsystem` (REV Spark Flex): intake/expel roller with direct percent and closed-loop RPM methods. CAN 14.
- `ConveyorSubsystem` (REV Spark Flex): moves game pieces toward/away from the shooter, with percent and closed-loop RPM control. CAN 16; inverted and brake idle mode are configured in the subsystem.
- `ShooterFlywheelSubsystem` (two CTRE TalonFX flywheel motors plus REV Spark Flex feed wheel): flywheel velocity control, distance-to-RPM interpolation, at-speed/readiness filtering, feed-wheel health checks, telemetry, and SysId characterization. Flywheel leader/follower CAN 32/33; feed wheel CAN 19.
- `BlinkinLedSubsystem` (REV Blinkin LED controller): shows robot mode and hub-tag/search/alignment/error states. PWM 1.
- `Telemetry`: drivetrain telemetry publisher owned by `RobotContainer` (not a motor subsystem).

## Driver and Auxiliary Controls
See `RobotContainer.configureBindings()` for the authoritative mapping; comments and old bindings in the file may be disabled.

- Driver left stick: field-centric translation; right X: manual rotation.
- Driver POV up/down: hold a driver-relative heading while retaining stick translation.
- Driver A: field-orientation reset, using Limelight heading if available and otherwise gyro seed; driver Y: gyro-seed reset without vision.
- Driver X: hold `AlignToFuelHubCommand` (bounded by the configured timeout).
- Test mode only, driver B/LB/RB/Start: shooter SysId quasistatic forward/reverse and dynamic forward/reverse.
- Auxiliary right stick Y: manual conveyor-arm movement; auxiliary Y/A: arm high/low endpoint requests.
- Auxiliary right/left trigger: intake roller forward/reverse RPM.
- Auxiliary right bumper: spin shooter to distance-derived RPM; left bumper: feed conveyor when shooter is ready.
- Auxiliary Back: distance-based shooter/feed command.
- Auxiliary Start: spin shooter at preset RPM.
- Auxiliary X: run conveyor at a fixed RPM; POV right/left: run conveyor forward/reverse by percent.
- Pit/debug calibration controls require the configured eligible profile and tuning-armed state. The climber bindings and climber hardware are not active in this branch.

## Vision and Localization
- Front Limelight NetworkTables name: `limelight`; AprilTag pipeline index is configured in `Constants.Vision.Front`.
- `AlignToFuelHubCommand` reads alliance-appropriate Limelight pose estimates, requires at least two tags for a fresh target, and briefly latches a recently observed heading.
- Hub-distance calculation in `RobotContainer` uses the hub-tag robot-space estimate, can latch a recent valid distance for a short interval, and maps distance through the shooter's distance/RPM lookup table.
- `CommandSwerveDrivetrain` supports `tv`, `tid`, `botpose_wpiblue`, `tl`, and `cl` for vision/odometry features. However, `Robot` currently keeps vision odometry disabled in all mode init callbacks; alignment and hub-distance use should not be confused with pose-estimator fusion.

## PID and Feedforward Tuning Checklist
Values are starting/configuration values, not a guarantee of good behavior on the current robot. Tune safely on the robot, one loop at a time, and confirm sensor direction, limits, and mechanisms before increasing output. Most gains below are compile-time values; only the shooter distance RPM trim has a dashboard draft/apply path, gated to pit tuning.

| Loop / constants | Current location and values | What to validate/tune |
| --- | --- | --- |
| Swerve steer | `generated/TunerConstants.java`: `steerGains` (`kP=100`, `kI=0`, `kD=0.5`, `kS=0.1`, `kV=1.79`, `kA=0`) | Tune module steering response and feedforward; check for oscillation, lag, and module alignment. |
| Swerve drive | `generated/TunerConstants.java`: `driveGains` (`kP=0.1`, `kI=0`, `kD=0`, `kS=0`, `kV=0.124`) | Validate velocity tracking and acceleration under load; also verify `kSpeedAt12Volts`, wheel radius, gear/coupling ratios, offsets, and `kSlipCurrent` against the actual robot. |
| Fuel-hub heading | `AlignToFuelHubCommand.headingPid` in `commands/AlignToFuelHubCommand.java`: `(5.0, 0.0, 0.1)`; output limit 5 rad/s | Tune heading response and stability against real Limelight/pose latency. The gains are literals in the command, not `Constants`. |
| Field-centric X/Y and heading | `AutoAlignFieldCentric`: X/Y `(1.6, 0, 0)`; rotation `(4.0, 0, 0)`; motion/profile constraints are adjacent | Tune position convergence, overshoot, and motion constraints on the field. These gains/tolerances are command literals. |
| Driver heading hold | `RobotContainer` POV controller: `(5.0, 0.0, 0.1)` | Validate heading hold and snap behavior; tolerance comes from `HEADING_HOLD_TOLERANCE_DEG`. |
| Conveyor-arm position hold | `Constants.ConveyorArm.POSITION_KP_VOLTS_PER_DEGREE=0.144`, `POSITION_KI_VOLTS_PER_DEGREE_SECOND=0`, `POSITION_KD_VOLTS_PER_DEGREE_PER_SECOND=0`; `HOLD_KG_VOLTS=0` | Tune position response and gravity hold only after encoder offset/direction, horizontal reference, real low/high limits, and motor directions are verified. Current arm limits (`0`/`360` degrees) are placeholders; configure safe measured limits before presets. |
| Shooter flywheel velocity | `Constants.ShooterTune.FLYWHEEL_SLOT0_KP_DEFAULT=2.0`; slot I/D/KS/KV/KA defaults are zero. Primary runtime uses TalonFX velocity torque-current FOC, with VelocityVoltage fallback. | Tune the active TalonFX slot gains/feedforward for the deployed control mode. Validate fallback separately if it can be used; do not assume the REV `PID_P_DEFAULT` is the flywheel PID. SysId can inform feedforward characterization. |
| Shooter feed wheel velocity | `Constants.ShooterTune.FEED_WHEEL_PID_P_DEFAULT=0.0005`, I/D `0`, `FEED_WHEEL_PID_FF_DEFAULT=0.00175` | Tune wheel speed tracking and verify motor-to-wheel ratio, direction, current limit, and target RPM. Feed-wheel health gating depends on achieving commanded speed. |
| Intake roller velocity | `IntakeRollerSubsystem` config literals: `P=0.00045`, `I=0`, `D=0`, velocity FF `0.00020` | Tune if RPM commands do not track; gains currently live in the subsystem, not `Constants`. |
| Conveyor velocity | `ConveyorSubsystem` config literals: `P=0.0031`, `I=0`, `D=0`, velocity FF `0.00020` | Tune if RPM commands do not track; verify direction, brake behavior, and mechanism load. |
| Distance-to-shot table (not PID) | `Constants.ShooterTune.DISTANCE_POINT_METERS_DEFAULTS` and `RPM_POINT_DEFAULTS` | Calibrate actual hub distances and shot RPMs; this lookup/interpolation is often the relevant tuning for distance-based shooting, separate from the flywheel PID. |

## Build and Dependency Notes
- Java 17.
- GradleRIO plugin version is defined in `build.gradle`.
- Desktop simulation is configured.
- Review `build.gradle` for current jar packaging and deploy settings; avoid relying on old packaging notes without checking the active build configuration.

## Current Gaps / Risks
- Several motor gains and swerve module parameters need real-robot validation; values are not all centralized in `Constants.java`.
- Conveyor-arm encoder offset/reference and physical low/high limits must be measured and set before using endpoint presets; the current full-range values are placeholders.
- Vision odometry is intentionally disabled in `Robot.java` at present. Hub alignment and shooting distance use vision through separate paths.
- Some command classes, PathPlanner event names, and controller comments are legacy/compatibility paths. `RobotContainer.configureBindings()` and named-command registrations determine what is actually active.
- There is no active climber subsystem in this branch; older documentation referring to a climber or ratchet is stale.
