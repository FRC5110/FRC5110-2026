package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class SpinShooterAtRpmCommand extends Command {
    private static final double MIN_VALID_TARGET_RPM = 1e-3;

    private final ShooterFlywheelSubsystem shooter;
    private final DoubleSupplier rpmSupplier;
    private final Runnable updateLedAction;
    private final Runnable clearLedAction;
    private double latchedTargetRpm = 0.0;

    public SpinShooterAtRpmCommand(
        ShooterFlywheelSubsystem shooter,
        DoubleSupplier rpmSupplier
    ) {
        this(shooter, rpmSupplier, () -> {}, () -> {});
    }

    public SpinShooterAtRpmCommand(
        ShooterFlywheelSubsystem shooter,
        DoubleSupplier rpmSupplier,
        Runnable updateLedAction,
        Runnable clearLedAction
    ) {
        this.shooter = shooter;
        this.rpmSupplier = rpmSupplier;
        this.updateLedAction = updateLedAction;
        this.clearLedAction = clearLedAction;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        latchedTargetRpm = 0.0;
        updateLatchedTargetRpm();
        updateLedAction.run();
        SmartDashboard.putString("Shooter/FeedState", "SpinOnlySpinningUp");
        SmartDashboard.putString("AutoShoot/State", "SpinOnlySpinningUp");
    }

    @Override
    public void execute() {
        updateLatchedTargetRpm();
        shooter.setTargetRPM(latchedTargetRpm);
        updateLedAction.run();

        String state;
        if (shooter.isUsingBoost()) {
            state = "SpinOnlyBoosting";
        } else if (shooter.atSpeed()) {
            state = "SpinOnlyAtSpeed";
        } else {
            state = "SpinOnlySpinningUp";
        }

        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putNumber("AutoShoot/SpinOnlyTargetRPM", targetRpm);
        // SmartDashboard.putString("Shooter/FeedState", state);
        // SmartDashboard.putString("AutoShoot/State", state);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        latchedTargetRpm = 0.0;
        clearLedAction.run();
        SmartDashboard.putString("Shooter/FeedState", "Idle");
        SmartDashboard.putString("AutoShoot/State", interrupted ? "SpinOnlyInterrupted" : "SpinOnlyComplete");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void updateLatchedTargetRpm() {
        double candidateTargetRpm = rpmSupplier.getAsDouble();
        if (Double.isFinite(candidateTargetRpm) && Math.abs(candidateTargetRpm) > MIN_VALID_TARGET_RPM) {
            latchedTargetRpm = candidateTargetRpm;
        }
    }
}
