package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class CalibrateShooterDistanceCommand extends Command {
    private static final String CALIBRATION_STATE_KEY = "AutoShoot/ShooterCalibrationState";
    private static final String LATCHED_DISTANCE_KEY = "AutoShoot/ShooterCalibrationLatchedDistanceMeters";
    private static final String LATCHED_SOURCE_KEY = "AutoShoot/ShooterCalibrationLatchedDistanceSource";
    private static final String SUGGESTED_RPM_KEY = "AutoShoot/ShooterCalibrationSuggestedRPM";
    private static final String CURRENT_TARGET_RPM_KEY = "AutoShoot/ShooterCalibrationCurrentTargetRPM";
    private static final String CURRENT_RPM_KEY = "AutoShoot/ShooterCalibrationCurrentRPM";
    private static final String AT_SPEED_KEY = "AutoShoot/ShooterCalibrationAtSpeed";
    private static final String CAN_FEED_KEY = "AutoShoot/ShooterCalibrationCanFeed";
    private static final String FEED_ENABLED_KEY = "AutoShoot/ShooterCalibrationFeedEnabled";
    private static final String FORCE_FEED_KEY = "AutoShoot/ShooterCalibrationForceFeed";
    private static final String READY_TO_FEED_KEY = "AutoShoot/ShooterCalibrationReadyToFeed";
    private static final String USING_BOOST_KEY = "AutoShoot/ShooterCalibrationUsingBoost";

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier distanceSupplier;
    private final Supplier<String> distanceSourceSupplier;
    private final DoubleSupplier targetRpmSupplier;
    private final BooleanSupplier feedEnabledSupplier;
    private final BooleanSupplier forceFeedSupplier;
    private final Runnable updateLedAction;
    private final Runnable clearLedAction;
    private final ShooterFeedGate feedGate = new ShooterFeedGate();

    private double latchedDistanceMeters = Double.NaN;
    private String latchedDistanceSource = "Unknown";
    private boolean hasLatchedDistance = false;

    public CalibrateShooterDistanceCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier distanceSupplier,
        Supplier<String> distanceSourceSupplier,
        DoubleSupplier targetRpmSupplier,
        BooleanSupplier feedEnabledSupplier,
        BooleanSupplier forceFeedSupplier,
        Runnable updateLedAction,
        Runnable clearLedAction
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.distanceSupplier = distanceSupplier;
        this.distanceSourceSupplier = distanceSourceSupplier;
        this.targetRpmSupplier = targetRpmSupplier;
        this.feedEnabledSupplier = feedEnabledSupplier;
        this.forceFeedSupplier = forceFeedSupplier;
        this.updateLedAction = updateLedAction;
        this.clearLedAction = clearLedAction;

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        latchedDistanceMeters = distanceSupplier.getAsDouble();
        latchedDistanceSource = distanceSourceSupplier.get();
        hasLatchedDistance = Double.isFinite(latchedDistanceMeters) && latchedDistanceMeters > 0.0;
        feedGate.reset();

        double suggestedRpm = hasLatchedDistance
            ? shooter.getRecommendedRPMForDistanceMeters(latchedDistanceMeters)
            : 0.0;

        conveyor.stop();
        updateLedAction.run();

        SmartDashboard.putString(CALIBRATION_STATE_KEY, hasLatchedDistance ? "Running" : "NoLimelightDistance");
        SmartDashboard.putNumber(LATCHED_DISTANCE_KEY, latchedDistanceMeters);
        SmartDashboard.putString(LATCHED_SOURCE_KEY, latchedDistanceSource);
        SmartDashboard.putNumber(SUGGESTED_RPM_KEY, suggestedRpm);
        SmartDashboard.putBoolean(CAN_FEED_KEY, false);
        SmartDashboard.putBoolean(FORCE_FEED_KEY, false);
        SmartDashboard.putBoolean(READY_TO_FEED_KEY, false);
    }

    @Override
    public void execute() {
        updateLedAction.run();

        if (!hasLatchedDistance) {
            latchedDistanceMeters = distanceSupplier.getAsDouble();
            latchedDistanceSource = distanceSourceSupplier.get();
            hasLatchedDistance = Double.isFinite(latchedDistanceMeters) && latchedDistanceMeters > 0.0;

            double suggestedRpm = hasLatchedDistance
                ? shooter.getRecommendedRPMForDistanceMeters(latchedDistanceMeters)
                : 0.0;

            SmartDashboard.putNumber(LATCHED_DISTANCE_KEY, latchedDistanceMeters);
            SmartDashboard.putString(LATCHED_SOURCE_KEY, latchedDistanceSource);
            SmartDashboard.putNumber(SUGGESTED_RPM_KEY, suggestedRpm);

            if (hasLatchedDistance) {
                SmartDashboard.putString(CALIBRATION_STATE_KEY, "Running");
            }
        }

        if (!hasLatchedDistance) {
            shooter.stop();
            conveyor.stop();
            feedGate.reset();
            SmartDashboard.putString(CALIBRATION_STATE_KEY, "NoLimelightDistance");
            SmartDashboard.putBoolean(AT_SPEED_KEY, false);
            SmartDashboard.putBoolean(CAN_FEED_KEY, false);
            SmartDashboard.putBoolean(FORCE_FEED_KEY, false);
            SmartDashboard.putBoolean(READY_TO_FEED_KEY, false);
            SmartDashboard.putBoolean(USING_BOOST_KEY, false);
            SmartDashboard.putNumber(CURRENT_TARGET_RPM_KEY, 0.0);
            SmartDashboard.putNumber(CURRENT_RPM_KEY, shooter.getCurrentRPM());
            SmartDashboard.putBoolean(FEED_ENABLED_KEY, feedEnabledSupplier.getAsBoolean());
            return;
        }

        double targetRpm = targetRpmSupplier.getAsDouble();
        if (!Double.isFinite(targetRpm)) {
            targetRpm = 0.0;
        }

        shooter.setTargetRPM(targetRpm);

        double feedPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_FEED_POWER_DEFAULT, -1.0, 1.0);
        double holdPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_HOLD_POWER_DEFAULT, -1.0, 1.0);
        boolean autoFeedEnabled = feedEnabledSupplier.getAsBoolean();
        boolean forceFeed = forceFeedSupplier.getAsBoolean();
        boolean usingBoost = shooter.isUsingBoost();
        double currentRpm = Math.abs(shooter.getCurrentRPM());
        boolean readyToFeed = shooter.isReadyToFeed();
        boolean autoCanFeed;

        if (!autoFeedEnabled || forceFeed) {
            feedGate.reset();
            autoCanFeed = false;
        } else {
            autoCanFeed = feedGate.shouldFeed(shooter);
        }

        boolean canFeed = forceFeed || autoCanFeed;

        if (canFeed) {
            conveyor.run(feedPower);
        } else {
            conveyor.run(holdPower);
        }

        String calibrationState;
        if (forceFeed) {
            calibrationState = "ManualFeed";
        } else if (autoCanFeed) {
            calibrationState = "AutoPacketFeeding";
        } else if (shooter.shouldPauseFeedWhileBoosting()) {
            calibrationState = "BoostHolding";
        } else if (!autoFeedEnabled) {
            calibrationState = "FeedDisabled";
        } else {
            calibrationState = "SpinningUp";
        }

        SmartDashboard.putString(CALIBRATION_STATE_KEY, calibrationState);
        SmartDashboard.putNumber(CURRENT_TARGET_RPM_KEY, targetRpm);
        SmartDashboard.putNumber(CURRENT_RPM_KEY, currentRpm);
        SmartDashboard.putBoolean(AT_SPEED_KEY, shooter.atSpeed());
        SmartDashboard.putBoolean(CAN_FEED_KEY, canFeed);
        SmartDashboard.putBoolean(FEED_ENABLED_KEY, autoFeedEnabled);
        SmartDashboard.putBoolean(FORCE_FEED_KEY, forceFeed);
        SmartDashboard.putBoolean(READY_TO_FEED_KEY, readyToFeed);
        SmartDashboard.putBoolean(USING_BOOST_KEY, usingBoost);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        shooter.stop();
        feedGate.reset();
        clearLedAction.run();
        SmartDashboard.putString(
            CALIBRATION_STATE_KEY,
            interrupted ? "Interrupted" : "Idle"
        );
        SmartDashboard.putBoolean(CAN_FEED_KEY, false);
        SmartDashboard.putBoolean(FORCE_FEED_KEY, false);
        SmartDashboard.putBoolean(READY_TO_FEED_KEY, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
