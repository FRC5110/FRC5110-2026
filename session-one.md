# Session One Notes

Last updated: 2026-03-12

## Purpose

This file captures the current operator-facing state of the robot so the next test session can start quickly without re-reading code.

## Main Operator Changes In This Session

- Added a dedicated no-vision field reset on driver `Y`.
- Kept driver `A` as the vision-preferred field reset.
- Moved hub-distance calibration to AUX `B`.
- Added AUX `D-pad up/down` for manual intake arm jog.
- Added AUX `D-pad right/left` for manual conveyor jog.
- Added a dashboard-driven shooter calibration mode on AUX `Left stick click`.
- Kept distance-based shooting and hub alignment Limelight-only for now.

## Follow-Up Changes In This Session

- Shooter flywheel tuning was updated for the new dual-motor setup:
  - lower default `P`
  - default `D = 0`
  - explicit follower direction constant
  - wider shared feed-ready window so conveyor commands do not disagree about when the shooter is ready
- Shooter distance interpolation now starts at `0.0 m`; negative placeholder distance points were removed from `Constants.ShooterTune`.
- Shooter calibration target RPM is now human-owned only:
  - the robot no longer auto-seeds or nudges the target RPM entry
  - the `+/-50 RPM` and `Use Suggested` dashboard actions were removed
  - `Shooter Cal Target RPM` is no longer persistent, so it should stay editable
- Teleop heading behavior was split cleanly:
  - robot/mechanical front stays the shooter/Limelight side
  - driver heading snap now feels intake-front
  - vision and shooting commands still use the real shooter front
  - code changes for this are tagged with `DEBUG_TELEOP_FRONT`

## Current Driver Controls

- `Left stick`: drive translation
- `Right stick`: heading/snap control, intake-front from the driver point of view
- `A`: reset field orientation with vision if available
- `Y`: reset field orientation without vision
- `D-pad up`: snap so the intake faces away from the driver
- `D-pad down`: snap so the intake faces toward the driver

## Current AUX Controls

- `Right stick Y`: manual intake arm control
- `RT`: intake roller forward
- `LT`: intake roller reverse
- `RB`: spin shooter at distance-based RPM
- `LB`: feed conveyor when shooter is ready
- `Back`: auto distance shoot + feed
- `Left stick click`: shooter-distance calibration mode
- `Start`: preset shooter spin
- `X`: conveyor forward at fixed RPM
- `Y`: intake arm top preset
- `B`: hub-distance calibration
- `A`: intake arm bottom preset
- `D-pad up/down`: manual intake arm jog
- `D-pad right/left`: manual conveyor jog

## Shooter Calibration Workflow

1. Place the robot at a measured spot in front of the hub.
2. Make sure the hub-tag indicator is green.
3. Use AUX `Left stick click` to start shooter-distance calibration mode.
4. Read the latched distance on the dashboard.
5. Adjust the dashboard shooter calibration RPM until the shot is correct.
6. Only the human tester should edit `Shooter Cal Target RPM`; the robot no longer writes back into that entry.
7. Record the pair: `distance, RPM`.
8. Move to a new spot and repeat until enough points exist for the interpolation table.

## Important Current Behavior

- Hub distance for shooting is taken from Limelight hub tags only.
- If no valid hub tags are visible, the robot does not fall back to odometry for shot distance.
- Distance-based shot commands latch the chosen RPM once at command start.
- Driver `A` still applies a vision-based field heading reset when a valid hub tag solve exists.
- Driver `Y` still makes the robot's current heading the field-oriented forward reference.
- Right stick and driver `D-pad up/down` now use intake-front teleop snap logic, while shooter alignment remains shooter-front.
- Shooter calibration target RPM should remain directly editable on Shuffleboard; if an old cached widget misbehaves, delete it once and let the robot recreate it.
- Teleop LED default is solid orange.
- Autonomous LED default is blinking orange/gold.

## Build Verification

- `./gradlew build` passed after the shooter tuning, calibration widget cleanup, and teleop heading updates.

## Recommended Next Step

- After collecting enough `distance, RPM` pairs, update the shooter interpolation table in `Constants.java` and verify with `AutoGetDistanceShootFeedCommand`.
