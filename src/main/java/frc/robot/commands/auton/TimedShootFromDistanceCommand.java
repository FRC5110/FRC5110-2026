package frc.robot.commands.auton;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ShooterFeedGate;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class TimedShootFromDistanceCommand extends Command {

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier distanceMetersSupplier;
    private final double conveyorPercent;
    private final double durationSec;
    private final Timer timer = new Timer();
    private final ShooterFeedGate feedGate = new ShooterFeedGate();

    public TimedShootFromDistanceCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier distanceMetersSupplier,
        double conveyorPercent,
        double durationSec
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.distanceMetersSupplier = distanceMetersSupplier;
        this.conveyorPercent = MathUtil.clamp(conveyorPercent, -1.0, 1.0);
        this.durationSec = Math.max(0.0, durationSec);

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        feedGate.reset();
        timer.restart();
        conveyor.stop();
        SmartDashboard.putString("AutoShoot/State", "TimedSpinUp");
    }

    @Override
    public void execute() {
        double distanceMeters = distanceMetersSupplier.getAsDouble();
        shooter.setTargetFromDistanceMeters(distanceMeters);
        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putNumber("AutoShoot/HubDistanceMeters", distanceMeters);

        double holdPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_HOLD_POWER_DEFAULT, -1.0, 1.0);
        boolean canFeed = feedGate.shouldFeed(shooter);

        if (canFeed) {
            conveyor.run(conveyorPercent);
            // Realtime dashboard publishing disabled to reduce loop overflows.
            // SmartDashboard.putString("AutoShoot/State", "TimedFeeding");
        } else {
            conveyor.run(holdPower);
            // Realtime dashboard publishing disabled to reduce loop overflows.
            // SmartDashboard.putString("AutoShoot/State", shooter.isUsingBoost() ? "TimedBoosting" : "TimedSpinUp");
        }

        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putNumber("AutoShoot/DurationSec", durationSec);
        // SmartDashboard.putNumber("AutoShoot/ElapsedSec", timer.get());
        // SmartDashboard.putNumber("AutoShoot/RemainingSec", Math.max(0.0, durationSec - timer.get()));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        feedGate.reset();
        conveyor.stop();
        shooter.stop();
        SmartDashboard.putString("AutoShoot/State", interrupted ? "TimedInterrupted" : "TimedComplete");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSec);
    }
}
