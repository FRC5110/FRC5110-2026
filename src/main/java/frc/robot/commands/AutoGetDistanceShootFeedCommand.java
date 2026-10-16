package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class AutoGetDistanceShootFeedCommand extends Command {
    private static final double MIN_VALID_TARGET_RPM = 1e-3;

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier rpmSupplier;
    private final Runnable updateLedAction;
    private final Runnable clearLedAction;
    private final ShooterFeedGate feedGate = new ShooterFeedGate();
    private double latchedTargetRPM = 0.0;

    public AutoGetDistanceShootFeedCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier rpmSupplier
    ) {
        this(shooter, conveyor, rpmSupplier, () -> {}, () -> {});
    }

    public AutoGetDistanceShootFeedCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier rpmSupplier,
        Runnable updateLedAction,
        Runnable clearLedAction
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpmSupplier = rpmSupplier;
        this.updateLedAction = updateLedAction;
        this.clearLedAction = clearLedAction;

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        feedGate.reset();
        latchedTargetRPM = 0.0;
        updateLatchedTargetRPM();
        conveyor.stop();
        updateLedAction.run();
        SmartDashboard.putString("Shooter/FeedState", "AutoDistSpinningUp");
        SmartDashboard.putString("AutoShoot/State", "AutoDistSpinningUp");
    }

    @Override
    public void execute() {
        updateLatchedTargetRPM();
        shooter.setTargetRPM(latchedTargetRPM);
        //updateLedAction.run();

        double feedPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_FEED_POWER_DEFAULT, -1.0, 1.0);
        double holdPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_HOLD_POWER_DEFAULT, -1.0, 1.0);
        boolean canFeed = feedGate.shouldFeed(shooter);

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
        shooter.stop();
        latchedTargetRPM = 0.0;
        clearLedAction.run();
        SmartDashboard.putString("Shooter/FeedState", "Idle");
        SmartDashboard.putString("AutoShoot/State", interrupted ? "AutoDistInterrupted" : "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void updateLatchedTargetRPM() {
        double candidateTargetRPM = rpmSupplier.getAsDouble();
        if (Double.isFinite(candidateTargetRPM) && Math.abs(candidateTargetRPM) > MIN_VALID_TARGET_RPM) {
            latchedTargetRPM = candidateTargetRPM;
        }
    }
}
