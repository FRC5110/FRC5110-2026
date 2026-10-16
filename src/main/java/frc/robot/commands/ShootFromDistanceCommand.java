package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShootFromDistanceCommand extends Command {

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier distanceMetersSupplier;
    private final double conveyorPercent;
    private final ShooterFeedGate feedGate = new ShooterFeedGate();

    public ShootFromDistanceCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier distanceMetersSupplier,
        double conveyorPercent
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.distanceMetersSupplier = distanceMetersSupplier;
        this.conveyorPercent = MathUtil.clamp(conveyorPercent, 0.0, 1.0);

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        feedGate.reset();
        conveyor.stop();
        SmartDashboard.putString("Shooter/FeedState", "DistanceSpinningUp");
        SmartDashboard.putString("AutoShoot/State", "DistanceSpinningUp");
    }

    @Override
    public void execute() {
        double presetRPM = Constants.ShooterTune.PRESET_SHOT_RPM_DEFAULT;
        shooter.setTargetRPM(presetRPM);
        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putNumber("AutoShoot/PresetShotRPM", presetRPM);

        double feedPower = conveyorPercent;
        double holdPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_HOLD_POWER_DEFAULT, -1.0, 1.0);
        boolean usingBoost = shooter.isUsingBoost();
        boolean canFeed = feedGate.shouldFeed(shooter);

        String feedState;
        if (canFeed) {
            conveyor.run(feedPower);
            feedState = "DistanceFeeding";
        } else {
            conveyor.run(holdPower);
            if (usingBoost) {
                feedState = "DistanceBoosting";
            } else if (shooter.atSpeed()) {
                feedState = "DistanceReadyHolding";
            } else {
                feedState = "DistanceSpinningUp";
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
        SmartDashboard.putString("AutoShoot/State", interrupted ? "DistanceInterrupted" : "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
