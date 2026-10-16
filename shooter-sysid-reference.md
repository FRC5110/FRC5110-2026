# Shooter SysId Reference

Last updated: 2026-03-14

## Purpose

Use SysId on the shooter flywheel to identify the motor/feedforward model first, then tune closed-loop correction around that. For this robot, the main value is getting better `kS` and `kV` so the flywheel holds speed with less sag and less guesswork. SysId does not directly give the final `P/I/D`.

## Current Shooter Control In This Repo

- `src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java` uses Spark Flex velocity control in `runControlLoop()`.
- Current default gains in `src/main/java/frc/robot/Constants.java`:
  - `PID_P_DEFAULT = 0.006`
  - `PID_I_DEFAULT = 0.0`
  - `PID_D_DEFAULT = 0.0`
  - `FEEDFORWARD_KS_VOLTS_DEFAULT = 0.0`
  - `FEEDFORWARD_KV_VOLTS_PER_RPS_DEFAULT = 0.0078`
  - `FEEDFORWARD_KA_VOLTS_PER_RPS2_DEFAULT = 0.0`
  - `CURRENT_LIMIT_AMPS_DEFAULT = 60`
  - `MAX_RPM_DEFAULT = 6200.0`
- The subsystem now uses a hardware-safe compatibility path: Spark Flex velocity control plus `closedLoop.velocityFF(...)` derived from stored `kV`.
- Stored `kS` and `kA` values are kept in constants for reference and future software arbitrary-feedforward work, but are not applied device-side in the current shooter path.
- The subsystem also has a direct-voltage boost mode, but `BOOST_ENABLED_DEFAULT = false` right now.
- Shooter SysId bindings are test-mode only:
  - `Driver B`: quasistatic forward
  - `Driver LB`: quasistatic reverse
  - `Driver RB`: dynamic forward
  - `Driver Start`: dynamic reverse

## What SysId Is For Here

SysId gives three main values:

- `kS`: voltage needed to overcome static/friction effects
- `kV`: voltage needed per wheel speed
- `kA`: voltage needed per wheel acceleration

For this shooter:

- `kS` and `kV` are the first values to use
- `kA` is useful if arbitrary feedforward is applied in software
- `P` should then be tuned only to clean up error and recover from shot load
- `I` usually stays `0`
- `D` usually stays `0` or very small for a flywheel

## Where The Analyzer Is

The SysId analyzer is a desktop WPILib tool, not something on the roboRIO.

- VS Code: `WPILib: Start Tool` -> `SysId`
- Typical local install path: `~/wpilib/2026/tools`

The robot runs the characterization routine and records a `.wpilog`. That log is then opened in the SysId desktop tool for analysis.

## Recommended Characterization Workflow

When characterizing the shooter:

- do not feed balls
- disable or bypass the normal shooter velocity loop
- disable boost behavior
- drive the flywheel with direct voltage only
- keep the follower motor in follow mode
- log applied voltage, position, and velocity

For this subsystem, that means the SysId test path should temporarily bypass `runControlLoop()` and call `flywheelLeaderMotor.setVoltage(...)` directly.

Run the standard four tests:

1. quasistatic forward
2. quasistatic reverse
3. dynamic forward
4. dynamic reverse

## How To Use The Results

The practical tuning order is:

1. apply feedforward from SysId
2. retune `P`
3. leave `I = 0` unless there is a real steady-state bias
4. only add `D` if there is a specific measured reason
5. validate under actual shot load

### Current Path In This Repo

Keep Spark velocity control and use the hardware-safe compatibility path that only applies `kV` through `closedLoop.velocityFF(...)`.

Use SysId results like this:

- store `kS`, `kV`, and `kA` in `Constants.java`
- `kV` is actively applied in the current runtime path through `velocityFF(...)`
- `kS` and `kA` are recorded for future use, but do not affect the current shooter at runtime

This keeps the current shooter boot-safe on the deployed SPARK Flex hardware.

### Future Path: Arbitrary Feedforward In Software

Create a `SimpleMotorFeedforward(kS, kV, kA)` in software and compute feedforward volts each loop, then send that as arbitrary feedforward with the Spark velocity request.

That looks like this conceptually:

```java
double targetRps = activeTargetRPM / 60.0;
double ffVolts = shooterFeedforward.calculate(targetRps);

flywheelVelocityController.setSetpoint(
    activeTargetRPM,
    SparkFlex.ControlType.kVelocity,
    ClosedLoopSlot.kSlot0,
    ffVolts
);
```

This is the path to choose later if the team wants `kS`, `kV`, and `kA` all to affect runtime behavior directly.

## Important Unit Note

Be careful with units when moving SysId results into the current shooter path or a future software feedforward path.

If SysId identifies velocity in `rotations/sec`, then:

- SysId `kV` is in `V / (rot/s)`
- Spark velocity control usually works in `RPM`

So the conversion is:

- `kV_rev = kV_sysid / 60.0`

The same scaling idea applies to `kA` if acceleration units are being converted from per-second-based units to RPM-based units for a future software feedforward implementation:

- `kA_rev = kA_sysid / 60.0`

The current subsystem actively applies only the `kV` conversion in its compatibility configuration. `kA` scaling matters later if software feedforward is added.

## Recommended Tuning Sequence After SysId

1. keep boost disabled
2. apply `kV` in the current compatibility path and record `kS`/`kA` for later
3. test free-spin target tracking
4. increase `P` only until recovery from feed load is acceptable
5. re-test with real shots
6. only then re-enable and evaluate boost if extra recovery help is still needed

Boost should not be tuned first because it can hide whether the base velocity loop is actually correct.

## What To Change In This Repo When Implementing

If SysId work is added later, the expected code-touch areas are:

- `src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java`
  - add a SysId routine path
  - add a direct-voltage characterization method
  - bypass normal velocity control during the test
- `src/main/java/frc/robot/RobotContainer.java`
  - add temporary bindings for quasistatic/dynamic test commands if desired
- `src/main/java/frc/robot/Constants.java`
  - store updated feedforward values after characterization

## Bottom Line

For this shooter, use SysId primarily to replace guess-based `kV` tuning now and to capture `kS`/`kA` for a later software-feedforward upgrade. Once the active feedforward is correct, tune `P` for disturbance recovery under load. Do not expect SysId to directly produce the final closed-loop gains by itself.
