# Controller Bindings

Last updated: 2026-03-12

## Driver Controller

- `Left stick`: field-centric translation drive
- `Right stick`: heading/snap direction input
- `A`: reset field orientation with vision if available
- `Y`: reset field orientation without vision
- `D-pad up`: snap to `0 deg`
- `D-pad down`: snap to `180 deg`

Currently unused on driver:

- `B`
- `X`
- `LB`
- `RB`
- `LT`
- `RT`
- `Back`
- `Start`
- `Left stick click`
- `Right stick click`
- `D-pad left`
- `D-pad right`

## AUX Controller

- `Right stick Y`: manual intake arm control
- `RT`: intake roller forward at `5000 RPM`
- `LT`: intake roller reverse at `-3000 RPM`
- `RB`: spin shooter at distance-based RPM
- `LB`: feed conveyor when shooter is ready
- `Back`: run `AutoGetDistanceShootFeedCommand`
- `Left stick click`: run `CalibrateShooterDistanceCommand`
- `Start`: spin shooter at preset shot RPM
- `X`: run conveyor forward at `3000 RPM`
- `Y`: intake arm top preset
- `B`: run hub-distance calibration
- `A`: intake arm bottom preset
- `D-pad up`: manual intake arm jog up while held
- `D-pad down`: manual intake arm jog down while held
- `D-pad right`: conveyor forward while held
- `D-pad left`: conveyor backward while held

Currently unused on AUX:

- `Right stick click`
- `Right stick X`

## Notes

- Distance-based shooting uses Limelight hub-tag distance only.
- Distance-based shooter commands latch their RPM target once at command start.
- Hub-tag LED feedback turns green when valid hub tags are visible and red when they are not during shot-distance and calibration workflows.
