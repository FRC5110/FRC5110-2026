# Shooter Distance Reference

`drivetrain.getPose()` itself is just a read. The thing that was disturbing driver control was vision fusion changing the pose estimate, which is why teleop currently keeps it off in `Robot.java`.

Given that constraint, the best shooting approach is:

- Keep teleop vision odometry off.
- Stop using raw `botpose_wpiblue` for shooter distance.
- Use a separate shooter-only Limelight measurement that does not feed the drivetrain pose estimator.
- Filter it, validate it, and fall back to odometry or a preset if vision is bad.

With your numbers, `botpose`-based distance looks wrong:

- Real distance: `2.3 m`
- Current shot distance used: `8.09 m`
- `avgTagDist`: `2.64 m`

That says the current pose-based branch in `RobotContainer.java` is not trustworthy for shooting right now. Practically, the better move is to switch shooter distance to a dedicated Limelight scalar and tune shots off that. The simplest version is `avgTagDist` or `ty`; the more correct version is tag-relative distance to the hub tags and then convert to hub-center distance. For this robot as it sits today, `avgTagDist` is already much closer to reality than `8.09`, so that is the fastest path to a useful result.

If needed later, patch `ShootFeedNoAlign` to ignore `botpose` and use a dedicated vision-only shot distance with a clear dashboard value like `Shooter/DistanceUsedForRPM`.

## Shooter Tune Notes

The shooter is not currently live-tuned from dashboard reads. The subsystem uses compile-time `Constants.ShooterTune.*_DEFAULT` values directly, and that is intentional because prior live `SmartDashboard.get...` reads were contributing to loop overruns.

Given the current implementation, the best ways to improve time-to-target RPM are:

- Raise shooter current limit first. `50A` is a plausible acceleration bottleneck for spin-up.
- Reduce `PID_D`. The current `0.02` derivative gain is high for a flywheel and may be slowing rise and recovery.
- Then raise `PID_P` moderately so the velocity loop is more aggressive after boost hands off.
- If low-battery performance is still too soft, increase boost request and battery-based boost caps slightly.
- Only loosen `atSpeed` tolerance and stable-time gating after the actual acceleration behavior is improved.

Recommended starting test order:

1. `CURRENT_LIMIT_AMPS`: `50 -> 60`
2. `PID_D`: `0.02 -> 0.005` or `0.0`
3. `PID_P`: `0.050 -> 0.07`, then toward `0.09-0.10` if stable
4. `BOOST_VOLTS_REQUESTED`: `11.0 -> 11.5` or `12.0` if electrical margin allows
5. Low-battery boost caps: raise slightly if late-match spin-up remains slow

Important caveat:

- `PID_FF_DEFAULT = 0.00018` is currently limited by the subsystem's max-RPM clamp logic tied to `MAX_RPM_DEFAULT = 6200`, so increasing FF above roughly `0.000161` currently does nothing.
