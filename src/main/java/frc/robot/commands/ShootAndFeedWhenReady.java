package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShootAndFeedWhenReady extends Command {

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier rpmSupplier;
    private final ShooterFeedGate feedGate = new ShooterFeedGate();

    public ShootAndFeedWhenReady(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier rpmSupplier
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpmSupplier = rpmSupplier;

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        feedGate.reset();
        conveyor.stop();
        SmartDashboard.putString("Shooter/FeedState", "SpinningUp");
        SmartDashboard.putString("AutoShoot/State", "SpinningUp");
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
            feedState = "Feeding";
        } else {
            conveyor.run(holdPower);
            if (usingBoost) {
                feedState = "Boosting";
            } else if (shooter.atSpeed()) {
                feedState = "ReadyHolding";
            } else {
                feedState = "SpinningUp";
            }
        }

        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putString("Shooter/FeedState", feedState);
        // SmartDashboard.putString("AutoShoot/State", feedState);
    }

    @Override
    public void end(boolean interrupted) {
        feedGate.reset();
        conveyor.stop();
        shooter.stop();
        SmartDashboard.putString("Shooter/FeedState", "Idle");
        SmartDashboard.putString("AutoShoot/State", "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
