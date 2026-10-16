# Code Structure - 2026swervecode1

## Top-Level Layout
- `build.gradle`: GradleRIO build, deploy, and dependency config.
- `src/main/java/frc/robot`: robot code.
- `src/main/java/frc/robot/generated`: CTRE Tuner X generated drivetrain constants.
- `src/main/java/frc/robot/subsystems`: subsystem implementations.
- `src/main/java/frc/robot/commands`: command implementations used by `RobotContainer`.
- `vendordeps`: CTRE Phoenix6, REVLib, PathPlanner, PhotonVision, Grapple.

## Java Package Map
- `frc.robot.Main`: WPILib entrypoint.
- `frc.robot.Robot`: robot lifecycle and scheduler loop.
- `frc.robot.RobotContainer`: controller bindings and default drive behavior.
- `frc.robot.FieldGeometry`: 2026 field tag layout helpers and target pose math.
- `frc.robot.LimelightHelpers`: full Limelight helper utility class.
- `frc.robot.Telemetry`: publishes drivetrain/pose telemetry.
- `frc.robot.generated.TunerConstants`: generated CTRE swerve hardware and gain constants.
- `frc.robot.commands.AlignToClimbPostCommand`: aligns to climb post target with climb vision.
- `frc.robot.commands.AlignToFuelHubCommand`: aligns drivetrain to fuel hub target.
- `frc.robot.commands.ShootFromDistanceCommand`: distance-based shooter/conveyor command.
- `frc.robot.subsystems.*`: drivetrain/mechanism subsystems.

## Runtime Control Flow
1. `Main` starts `Robot`.
2. `Robot` constructs `RobotContainer`.
3. `RobotContainer` creates drivetrain (`TunerConstants.createDrivetrain()`) and sets default field-centric drive command.
4. `CommandScheduler` runs in `robotPeriodic()`.
5. `CommandSwerveDrivetrain.periodic()` applies alliance perspective and injects Limelight vision measurements into odometry.

## Subsystems and CAN IDs
- `CommandSwerveDrivetrain` (CTRE Phoenix 6 swerve)
  - Pigeon2: 13
  - Front Left: drive 5, steer 4, encoder 6
  - Front Right: drive 8, steer 7, encoder 9
  - Back Left: drive 2, steer 1, encoder 3
  - Back Right: drive 11, steer 10, encoder 12
- `IntakeRollerSubsystem` (Spark Flex): CAN 20
- `IntakeArmSubsystem` (Spark Flex + position hold): CAN 21
- `ConveyorSubsystem` (Spark Flex): CAN 22
- `ShooterLoaderSubsystem` (Spark Flex): CAN 23
- `ShooterFlywheelSubsystem` (Spark Flex velocity PID): CAN 24
- `ClimberSubsystem` (TalonFX + AndyMark ratchet servo): CAN 30, PWM 2
  - Servo latch behavior:
    - `engageLatch()` locks climber gear for ratchet hold.
    - `disengageLatch()` releases ratchet for motor movement.
    - `run(percent)` auto-releases latch and waits a short delay before driving motor.
    - `stopAndHold()` engages latch and stops motor output.

## Driver Controls in RobotContainer
- Left stick: field-centric translation.
- Right X: manual rotation.
- Hold `A`: orbit mode (heading PID around fixed hub point).
- POV: snap heading (0/90/180/270 deg).
- Hold `X`: `AlignToFuelHubCommand`.
- Hold `Y`: `AlignToClimbPostCommand`.
- Hold left trigger: align to fuel hub, then run `ShootFromDistanceCommand`.
- Hold right trigger: spin shooter flywheel to test RPM.

## Vision and Localization
- Limelight table: `limelight`.
- Inputs used for odometry correction:
  - `tv` (target valid)
  - `botpose_wpiblue` (6-element pose)
  - `tl` + `cl` (latency)
- Vision pose is latency-compensated and passed into drivetrain pose estimator.

## Build and Dependency Notes
- Java 17.
- GradleRIO plugin: `2026.2.1`.
- Desktop simulation configured.
- Fat jar packaging includes source backup and vendordeps backup.

## Current Gaps / Risks
- Several subsystem PID constants are marked as starting values and still need robot-side tuning.
- Climber latch constants are placeholders and require mechanism-side tuning:
  - `Constants.Climber.LATCH_ENGAGED_POSITION`
  - `Constants.Climber.LATCH_DISENGAGED_POSITION`
  - `Constants.Climber.LATCH_RELEASE_DELAY_SEC`
