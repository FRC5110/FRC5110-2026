# Shooter SysId Runbook

Last updated: 2026-03-14

## Purpose

This is the step-by-step procedure for running SysId on the shooter flywheel and copying the identified feedforward values back into the robot code.

The shooter SysId support is wired in here:

- `src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/Robot.java`

## Before You Start

- Remove all balls/fuel from the shooter and conveyor path.
- Put the robot in a safe area with clear space around the shooter.
- Deploy current code first.
- Use `test` mode only for these routines.
- WPILib datalogging is currently disabled by default in `Constants.Logging.WPILIB_DATA_LOGGING_ENABLED`
  to avoid a startup bootloop. Turn it back on before running SysId if you need a `.wpilog`.

## Driver Controller Bindings

These bindings only work in `test` mode.

- `B`: quasistatic forward
- `LB`: quasistatic reverse
- `RB`: dynamic forward
- `Start`: dynamic reverse

Hold the button for the full test. Releasing early cancels the routine.

## Recommended Test Order

Run the full set in this order:

1. quasistatic forward
2. quasistatic reverse
3. dynamic forward
4. dynamic reverse

You can run them in one logging session or separate sessions. SysId can analyze separate sections from the same log.

## What The Code Does During SysId

During the SysId routine, the shooter bypasses normal velocity control and directly applies the requested test voltage to the flywheel leader motor. The subsystem logs:

- applied voltage
- flywheel position
- flywheel velocity

The follower motor remains in follow mode.

## After The Test

Open the WPILib `SysId` desktop tool and load the recorded `.wpilog`.

Use the analyzed values for:

- `kS`
- `kV`
- `kA`

Then copy them into:

- `src/main/java/frc/robot/Constants.java`

Current shooter feedforward constants:

- `FEEDFORWARD_KS_VOLTS_DEFAULT`
- `FEEDFORWARD_KV_VOLTS_PER_RPS_DEFAULT`
- `FEEDFORWARD_KA_VOLTS_PER_RPS2_DEFAULT`

Current runtime note:

- the shooter compatibility path actively applies only `kV`
- `kS` and `kA` should still be stored in constants and docs for later software feedforward work
- after deployment, do not expect `kS` or `kA` changes alone to affect current shooter behavior

## Important Units Note

This repo stores shooter SysId feedforward constants in `rotations/sec` units.

If SysId reports:

- `kS` in volts
- `kV` in volts per rotations/sec
- `kA` in volts per rotations/sec^2

then those values can be copied directly into the constants above.

The subsystem currently converts and applies only `kV` into REV's default RPM-based `velocityFF(...)` controller term. `kS` and `kA` are kept for future software-feedforward work.

## After Updating The Constants

1. rebuild
2. redeploy
3. verify the shooter boots cleanly with no invalid REV parameter errors
4. test free-spin target tracking
5. test under real shot load
6. retune `PID_P_DEFAULT` only if needed for recovery under load

For this shooter, the main immediate gain from SysId is getting better active `kV`. `P` should only be doing cleanup and disturbance recovery after that.

## Current Default SysId Settings

These are defined in `src/main/java/frc/robot/Constants.java`:

- `SYSID_RAMP_RATE_VOLTS_PER_SEC_DEFAULT = 1.0`
- `SYSID_STEP_VOLTAGE_DEFAULT = 7.0`
- `SYSID_TIMEOUT_SEC_DEFAULT = 10.0`

## Quick Summary

- deploy code
- enter `test` mode
- hold the four shooter SysId buttons one at a time
- open the `.wpilog` in SysId
- copy `kS`, `kV`, and `kA` into `Constants.java`
- expect only `kV` to affect the current runtime path
- rebuild and retest
