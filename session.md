# Session Notes - 2026swervecode1

Last updated: 2026-03-08
Workspace: `c:\Users\RoboHerd 5110\Desktop\2024-25 code\2026swervecode1`

## Request Summary
- Add an AndyMark programmable servo to climber control.
- Update climber logic for ratchet-and-servo actuator kit behavior.
- Update markdown documentation to reflect current code and decisions.

## 2026 FRC Competition Snapshot
- Season theme: FIRST AGE.
- 2026 FRC game: REBUILT.
- Game objective (high level): alliances collect Fuel Pods and score them in structures such as Towers, Outposts, and Fuel Hubs, including converting Fuel Pods to Fuel and launching Fuel into targets.
- Match structure (high level): one autonomous period followed by teleoperated play.

## Competition Sources Used
- https://www.firstinspires.org/robotics/frc/game-and-season
- https://www.firstinspires.org/about/press-room/first-unveils-2026-first-robotics-competition-game-rebuilt
- https://www.frcmanual.com/2026/game-overview/

## Codebase Snapshot
- Framework: WPILib command-based robot (`Robot`, `RobotContainer`, subsystems).
- Language/tooling: Java 17, GradleRIO `2026.2.1`.
- Drivetrain stack: CTRE Phoenix 6 swerve (Tuner X generated constants + custom `CommandSwerveDrivetrain`).
- Vision stack: Limelight helpers + direct NetworkTables reads (`limelight/botpose_wpiblue`, `tv`, `tl`, `cl`) for pose correction.
- Mechanism stack:
  - REV Spark Flex: intake arm/roller, conveyor, shooter loader, shooter flywheel.
  - CTRE TalonFX + PWM servo: climber with ratchet latch support.

## Climber Ratchet/Servo Changes
- Added constants:
  - `Constants.PwmPorts.CLIMBER_LATCH_SERVO = 2`
  - `Constants.Climber.LATCH_ENGAGED_POSITION`
  - `Constants.Climber.LATCH_DISENGAGED_POSITION`
  - `Constants.Climber.LATCH_RELEASE_DELAY_SEC`
- `ClimberSubsystem` now owns a `Servo` for ratchet latch actuation.
- New climber API:
  - `engageLatch()`
  - `disengageLatch()`
  - `toggleLatch()`
  - `isLatchEngaged()`
- Behavioral update:
  - `stopAndHold()` now engages the ratchet latch and commands zero motor output.
  - `run(percent)` auto-disengages latch and waits release delay before powering the motor.
  - Startup defaults to engaged latch.

## Important Current State Notes
- `src/main/java/frc/robot/commands/` contains source-backed command files:
  - `AlignToClimbPostCommand`
  - `AlignToFuelHubCommand`
  - `ShootFromDistanceCommand`
- `RobotContainer` currently binds these commands and shooter test controls.
- `gradlew compileJava` passes after climber changes (with one TalonFX constructor deprecation warning).

## Files Reviewed This Session
- `build.gradle`
- `src/main/java/frc/robot/Main.java`
- `src/main/java/frc/robot/Robot.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/FieldGeometry.java`
- `src/main/java/frc/robot/LimelightHelpers.java`
- `src/main/java/frc/robot/Telemetry.java`
- `src/main/java/frc/robot/generated/TunerConstants.java`
- `src/main/java/frc/robot/subsystems/*.java`

## Suggested Next Session Starting Point
1. Tune climber latch positions and release delay on robot hardware.
2. Add operator bindings for explicit latch engage/disengage override if needed.
3. Validate behavior under load at climb top: latch engage timing, no backdrive, safe descent behavior.
4. Add a brief operator checklist for climb sequence and ratchet safety interlocks.

---

# Session Notes - 2026-03-08

Workspace: `c:\Users\RoboHerd 5110\Desktop\For-ROBOHerd3-main`

## Request Summary
- Diagnose PathPlanner alliance flipping so blue-authored paths mirror correctly on red.
- Implement a stable alliance source for auton instead of scattered blue fallbacks.
- Temporarily disable Limelight vision odometry during autonomous.
- Review likely sources of loop overruns without making additional code changes.

## Changes Implemented
- Added `src/main/java/frc/robot/AllianceUtil.java` as a sticky cached alliance helper.
- Updated alliance-dependent logic to use `AllianceUtil` in:
  - `src/main/java/frc/robot/Robot.java`
  - `src/main/java/frc/robot/RobotContainer.java`
  - `src/main/java/frc/robot/FieldGeometry.java`
  - `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`
  - `src/main/java/frc/robot/commands/AlignToClimbPostCommand.java`
  - `src/main/java/frc/robot/commands/DriveToClimbDistanceCommand.java`
- Kept PathPlanner's built-in red flip supplier, but routed it through `AllianceUtil::isRed`.
- Added one-shot auton diagnostics for:
  - `Auto/AllianceAtStart`
  - `Auto/ShouldFlipForRed`
  - `Auto/ResetPoseX`
  - `Auto/ResetPoseY`
  - `Auto/ResetPoseHeadingDeg`
  - `Auto/ResetPoseAlliance`
  - `Auto/ResetPoseFlipRed`
- Commented out autonomous vision odometry enable in `Robot.autonomousInit()` so Limelight odometry stays off during auton for now.

## Current PathPlanner / Alliance State
- PathPlanner autoflip was already wired through `AutoBuilder.configure(...)`.
- The main defect risk was alliance unknown-at-start behavior causing red autos to silently use blue logic.
- Alliance-sensitive commands and geometry helpers now resolve through the same cached alliance decision as PathPlanner.

## Loop Overrun Findings
- The issue does not look like constant full roboRIO saturation; it looks more like bursty stalls or update pressure.
- Highest-probability runtime hotspots:
  - Limelight pose + raw fiducial processing in `RobotContainer.getDistanceToFuelHubMeters()`
  - Raw fiducial processing in `AlignToFuelHubCommand`
  - CTRE swerve/CAN update pressure from the generated drivetrain defaults
- `Telemetry.telemeterize(...)` is not currently active because `m_robotContainer.updateTelemetry()` remains commented out in `Robot.robotPeriodic()`.
- Realtime SmartDashboard spam had already mostly been commented out before this session and was not identified as the primary overrun source.

## Suggested Next Test Order
1. Drive-only test with no shooter distance logic and no hub align.
2. Shooter spin using distance-based RPM path.
3. Hub-align command by itself.
4. Check CAN utilization and Phoenix diagnostics during the same tests.

## Build / Verification State
- `.\gradlew.bat compileJava` passed after the alliance refactor.
- `.\gradlew.bat compileJava` also passed after disabling autonomous vision odometry.
- Remaining warning:
  - deprecated `Command.schedule()` usage in `src/main/java/frc/robot/Robot.java`

## Shooter Tuning Analysis
- Shooter tuning is currently compile-time, not live dashboard-driven.
- The shooter subsystem uses `Constants.ShooterTune.*_DEFAULT` values directly and applies REV closed-loop gains during subsystem construction.
- Prior live `SmartDashboard.get...` reads were removed because they were contributing to loop overruns.
- Current shooter control strategy:
  - Boost phase uses direct voltage when the wheel is well below target and estimated time-to-target exceeds the handoff threshold.
  - Handoff phase uses Spark Flex velocity PID once the wheel is close enough to target.
  - Ready-to-feed state requires RPM to stay above `target - tolerance` for a short stable window.

## Shooter Tuning Recommendations
- Highest-value knobs for faster spin-up:
  - raise `CURRENT_LIMIT_AMPS`
  - reduce `PID_D`
  - then increase `PID_P`
  - then relax boost caps if battery sag behavior is still too conservative
- Recommended tune order:
  1. Increase shooter current limit from `50A` to about `60A`, then test `65-70A` only if temperatures and brownout margin remain acceptable.
  2. Reduce `PID_D` from `0.02` to around `0.005` or `0.0` to avoid slowing flywheel acceleration and recovery.
  3. Increase `PID_P` from `0.050` toward `0.07-0.10` after lowering D.
  4. If late-match spin-up still feels slow, raise boost request and low-battery boost caps slightly.
  5. Only after that, consider loosening ready-to-feed tolerance or stable time.
- Additional note:
  - `PID_FF_DEFAULT = 0.00018` is currently clamped by the subsystem's `MAX_RPM_DEFAULT = 6200`, so the effective feedforward ceiling is about `0.000161`. Increasing FF above that value currently has no effect.

## Shooter Control Observations
- Current defaults:
  - `P = 0.050`
  - `I = 0.0`
  - `D = 0.02`
  - `VelocityFF = 0.00018` before clamp
  - `CurrentLimit = 50A`
  - `BoostVoltsRequested = 11.0V`
  - `Boost enter/exit error = 75 / 25 RPM`
  - `AtSpeed tolerance = 100 RPM`
  - `AtSpeed stable time = 0.08 s`
- The subsystem appears more likely to be limited by current cap, conservative low-battery boost caps, and relatively high D gain than by lack of additional logic.
