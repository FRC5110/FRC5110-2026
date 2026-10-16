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

public class TimedShootAndFeedWhenReadyCommand extends Command {

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier rpmSupplier;
    private final double durationSec;
    private final Timer timer = new Timer();
    private final ShooterFeedGate feedGate = new ShooterFeedGate();

    public TimedShootAndFeedWhenReadyCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier rpmSupplier,
        double durationSec
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpmSupplier = rpmSupplier;
        this.durationSec = Math.max(0.0, durationSec);

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        feedGate.reset();
        timer.restart();
        conveyor.stop();
        SmartDashboard.putString("Shooter/FeedState", "TimedSpinningUp");
        SmartDashboard.putString("AutoShoot/State", "TimedSpinningUp");
        SmartDashboard.putNumber("AutoShoot/DurationSec", durationSec);
    }

    @Override
    public void execute() {
        shooter.setTargetRPM(rpmSupplier.getAsDouble());

        double feedPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_FEED_POWER_DEFAULT, -1.0, 1.0);
        double holdPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_HOLD_POWER_DEFAULT, -1.0, 1.0);
        boolean usingBoost = shooter.isUsingBoost();
        boolean canFeed = feedGate.shouldFeed(shooter);

        String feedState;
        if (canFeed) {
            conveyor.run(feedPower);
            feedState = "TimedFeeding";
        } else {
            conveyor.run(holdPower);
            if (usingBoost) {
                feedState = "TimedBoosting";
            } else if (shooter.atSpeed()) {
                feedState = "TimedReadyHolding";
            } else {
                feedState = "TimedSpinningUp";
            }
        }

        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putString("Shooter/FeedState", feedState);
        // SmartDashboard.putString("AutoShoot/State", feedState);
        // SmartDashboard.putNumber("AutoShoot/ElapsedSec", timer.get());
        // SmartDashboard.putNumber("AutoShoot/RemainingSec", Math.max(0.0, durationSec - timer.get()));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        feedGate.reset();
        conveyor.stop();
        shooter.stop();
        SmartDashboard.putString("Shooter/FeedState", "Idle");
        SmartDashboard.putString("AutoShoot/State", interrupted ? "TimedInterrupted" : "TimedComplete");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSec);
    }
}
