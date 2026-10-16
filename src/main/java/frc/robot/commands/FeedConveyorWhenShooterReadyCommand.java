package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class FeedConveyorWhenShooterReadyCommand extends Command {

    private final ConveyorSubsystem conveyor;
    private final ShooterFlywheelSubsystem shooter;
    private final ShooterFeedGate feedGate = new ShooterFeedGate();

    public FeedConveyorWhenShooterReadyCommand(
        ConveyorSubsystem conveyor,
        ShooterFlywheelSubsystem shooter
    ) {
        this.conveyor = conveyor;
        this.shooter = shooter;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        feedGate.reset();
    }

    @Override
    public void execute() {
        double feedPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_FEED_POWER_DEFAULT, -1.0, 1.0);
        double holdPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_HOLD_POWER_DEFAULT, -1.0, 1.0);
        boolean canFeed = feedGate.shouldFeed(shooter);

        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putBoolean("Shooter/ManualFeedReady", canFeed);
        // SmartDashboard.putNumber("Shooter/ManualFeedTargetRPM", shooter.getTargetRPM());
        // SmartDashboard.putNumber("Shooter/ManualFeedCurrentRPM", shooter.getCurrentRPM());

        if (canFeed) {
            conveyor.run(feedPower);
        } else {
            conveyor.run(holdPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feedGate.reset();
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
