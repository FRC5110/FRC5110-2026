package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.FieldGeometry;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToFuelHubCommand extends Command {

    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(1.0);
    private static final int REQUIRED_STABLE_CYCLES = 5;
    private static final double MAX_ROT_RAD_PER_SEC = 5.0;
    private static final double POSE_HEADING_LATCH_SEC = Constants.RobotContainer.FUEL_HUB_ALIGN_HEADING_LATCH_SEC;
    private static final double ACQUIRE_TIMEOUT_SEC = Constants.RobotContainer.FUEL_HUB_ALIGN_ACQUIRE_TIMEOUT_SEC;
    private static final double LOST_TARGET_TIMEOUT_SEC = Constants.RobotContainer.FUEL_HUB_ALIGN_LOST_TARGET_TIMEOUT_SEC;
    private static final int MIN_TAG_COUNT_TO_ACQUIRE = 2;
    private static final String FRONT_LIMELIGHT = Constants.Vision.Front.LIMELIGHT_NAME;

    private enum FinishReason {
        NONE,
        ALIGNED,
        ACQUIRE_TIMEOUT,
        LOST_TARGET
    }

    private final CommandSwerveDrivetrain drivetrain;
    private final Consumer<Boolean> hubTagStatusConsumer;
    private final Runnable clearLedAction;
    private final Runnable endAction;
    private final BooleanSupplier debugTelemetryEnabledSupplier;
    private final PIDController headingPid = new PIDController(5.0, 0.0, 0.1);

    private final SwerveRequest.FieldCentric holdAndTurnRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private int stableCycles = 0;
    private int[] hubTagIds = new int[0];
    private double lastGoodTargetHeadingRad = Double.NaN;
    private double lastGoodTargetHeadingTimestampSec = Double.NEGATIVE_INFINITY;
    private double lastGoodDistanceMeters = Double.NaN;
    private double startTimestampSec = Double.NEGATIVE_INFINITY;
    private double lastTargetHeadingTimestampSec = Double.NEGATIVE_INFINITY;
    private FinishReason finishReason = FinishReason.NONE;

    static record PoseAimTarget(
        boolean hasFreshPose,
        boolean usingLatchedHeading,
        double targetHeadingRad,
        double distanceMeters,
        int tagCount,
        String alignSource
    ) {
        boolean hasTargetHeading() {
            return Double.isFinite(targetHeadingRad);
        }
    }

    public AlignToFuelHubCommand(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, ignored -> {}, () -> {}, () -> {}, () -> false);
    }

    public AlignToFuelHubCommand(
        CommandSwerveDrivetrain drivetrain,
        Consumer<Boolean> hubTagStatusConsumer,
        Runnable clearLedAction
    ) {
        this(drivetrain, hubTagStatusConsumer, clearLedAction, () -> {}, () -> false);
    }

    public AlignToFuelHubCommand(
        CommandSwerveDrivetrain drivetrain,
        Consumer<Boolean> hubTagStatusConsumer,
        Runnable clearLedAction,
        Runnable endAction
    ) {
        this(drivetrain, hubTagStatusConsumer, clearLedAction, endAction, () -> false);
    }

    public AlignToFuelHubCommand(
        CommandSwerveDrivetrain drivetrain,
        Consumer<Boolean> hubTagStatusConsumer,
        Runnable clearLedAction,
        Runnable endAction,
        BooleanSupplier debugTelemetryEnabledSupplier
    ) {
        this.drivetrain = drivetrain;
        this.hubTagStatusConsumer = hubTagStatusConsumer;
        this.clearLedAction = clearLedAction;
        this.endAction = endAction;
        this.debugTelemetryEnabledSupplier =
            debugTelemetryEnabledSupplier == null ? () -> false : debugTelemetryEnabledSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        stableCycles = 0;

        headingPid.enableContinuousInput(-Math.PI, Math.PI);
        headingPid.reset();

        hubTagIds = FieldGeometry.getAllFuelHubTagIds();
        lastGoodTargetHeadingRad = Double.NaN;
        lastGoodTargetHeadingTimestampSec = Double.NEGATIVE_INFINITY;
        lastGoodDistanceMeters = Double.NaN;
        startTimestampSec = Timer.getFPGATimestamp();
        lastTargetHeadingTimestampSec = Double.NEGATIVE_INFINITY;
        finishReason = FinishReason.NONE;
        LimelightHelpers.SetFiducialIDFiltersOverride(FRONT_LIMELIGHT, hubTagIds);

        SmartDashboard.putString("AutoShoot/State", "Aligning");
        if (debugTelemetryEnabledSupplier.getAsBoolean()) {
            SmartDashboard.putString("AutoShoot/AlignSource", "Initializing");
        }
    }

    @Override
    public void execute() {
        double nowSec = Timer.getFPGATimestamp();
        Translation2d hubCenter = FieldGeometry.getFuelHubTrueCenter();
        LimelightHelpers.PoseEstimate poseEstimate = getAlliancePoseEstimate();
        PoseAimTarget poseAimTarget = selectPoseAimTarget(
            poseEstimate,
            hubCenter,
            nowSec,
            lastGoodTargetHeadingRad,
            lastGoodTargetHeadingTimestampSec,
            lastGoodDistanceMeters,
            POSE_HEADING_LATCH_SEC
        );

        double currentHeadingRad = drivetrain.getPose().getRotation().getRadians();
        double currentHeadingDeg = Math.toDegrees(currentHeadingRad);
        double targetHeadingDeg = Double.NaN;
        double headingErrorDeg = Double.NaN;
        double omega = 0.0;

        hubTagStatusConsumer.accept(poseAimTarget.hasFreshPose());

        if (poseAimTarget.hasTargetHeading()) {
            lastTargetHeadingTimestampSec = nowSec;
            targetHeadingDeg = Math.toDegrees(poseAimTarget.targetHeadingRad());
            double headingErrorRad = calculateHeadingErrorRad(currentHeadingRad, poseAimTarget.targetHeadingRad());
            headingErrorDeg = Math.toDegrees(headingErrorRad);

            omega = MathUtil.clamp(
                headingPid.calculate(currentHeadingRad, poseAimTarget.targetHeadingRad()),
                -MAX_ROT_RAD_PER_SEC,
                MAX_ROT_RAD_PER_SEC
            );

            if (poseAimTarget.hasFreshPose()) {
                lastGoodTargetHeadingRad = poseAimTarget.targetHeadingRad();
                lastGoodTargetHeadingTimestampSec = nowSec;
                lastGoodDistanceMeters = poseAimTarget.distanceMeters();
            }

            if (Math.abs(headingErrorRad) <= HEADING_TOLERANCE_RAD) {
                stableCycles++;
            } else {
                stableCycles = 0;
            }

            if (stableCycles >= REQUIRED_STABLE_CYCLES) {
                finishReason = FinishReason.ALIGNED;
            }
        } else {
            stableCycles = 0;

            if (lastTargetHeadingTimestampSec == Double.NEGATIVE_INFINITY) {
                if ((nowSec - startTimestampSec) >= ACQUIRE_TIMEOUT_SEC) {
                    finishReason = FinishReason.ACQUIRE_TIMEOUT;
                }
            } else if ((nowSec - lastTargetHeadingTimestampSec) >= LOST_TARGET_TIMEOUT_SEC) {
                finishReason = FinishReason.LOST_TARGET;
            }
        }

        drivetrain.setControl(
            holdAndTurnRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(omega)
        );

        publishTelemetry(
            poseEstimate,
            poseAimTarget,
            currentHeadingDeg,
            targetHeadingDeg,
            headingErrorDeg
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        endAction.run();
        clearLedAction.run();
        LimelightHelpers.SetFiducialIDFiltersOverride(FRONT_LIMELIGHT, new int[0]);
        SmartDashboard.putString("AutoShoot/State", getEndState(interrupted));
        if (debugTelemetryEnabledSupplier.getAsBoolean()) {
            SmartDashboard.putString("AutoShoot/AlignSource", getEndState(interrupted));
            SmartDashboard.putNumber("AutoShoot/AlignTxDeg", Double.NaN);
            SmartDashboard.putNumber("AutoShoot/AlignErrorDeg", Double.NaN);
            SmartDashboard.putNumber("AutoShoot/AlignDistanceMeters", Double.NaN);
            SmartDashboard.putNumber("AutoShoot/AlignDirectVisibleTagCount", 0);
            SmartDashboard.putBoolean("AutoShoot/AlignDirectLatched", false);
            SmartDashboard.putNumber("AutoShoot/AlignPoseTagCount", 0);
            SmartDashboard.putNumber("AutoShoot/AlignPoseX", Double.NaN);
            SmartDashboard.putNumber("AutoShoot/AlignPoseY", Double.NaN);
            SmartDashboard.putNumber("AutoShoot/AlignTargetHeadingDeg", Double.NaN);
            SmartDashboard.putNumber("AutoShoot/AlignCurrentHeadingDeg", Double.NaN);
            SmartDashboard.putNumber("AutoShoot/AlignHeadingErrorDeg", Double.NaN);
        }
    }

    @Override
    public boolean isFinished() {
        return finishReason != FinishReason.NONE;
    }

    private String getEndState(boolean interrupted) {
        if (interrupted) {
            return "Interrupted";
        }

        return switch (finishReason) {
            case ALIGNED -> "Aligned";
            case ACQUIRE_TIMEOUT -> "AcquireTimeout";
            case LOST_TARGET -> "LostTarget";
            case NONE -> "Idle";
        };
    }

    static PoseAimTarget selectPoseAimTarget(
        LimelightHelpers.PoseEstimate poseEstimate,
        Translation2d hubCenter,
        double nowSec,
        double lastGoodTargetHeadingRad,
        double lastGoodTargetHeadingTimestampSec,
        double lastGoodDistanceMeters,
        double latchSec
    ) {
        if (isUsablePoseEstimate(poseEstimate)) {
            double targetHeadingRad = calculateTargetHeadingRad(poseEstimate.pose, hubCenter);
            double distanceMeters = calculateDistanceMeters(poseEstimate.pose, hubCenter);
            return new PoseAimTarget(
                true,
                false,
                targetHeadingRad,
                distanceMeters,
                poseEstimate.tagCount,
                "LimelightPoseHubHeading"
            );
        }

        if (Double.isFinite(lastGoodTargetHeadingRad)
                && (nowSec - lastGoodTargetHeadingTimestampSec) <= latchSec) {
            return new PoseAimTarget(
                false,
                true,
                lastGoodTargetHeadingRad,
                lastGoodDistanceMeters,
                poseEstimate == null ? 0 : poseEstimate.tagCount,
                "LimelightPoseHubHeadingLatched"
            );
        }

        return new PoseAimTarget(
            false,
            false,
            Double.NaN,
            Double.NaN,
            poseEstimate == null ? 0 : poseEstimate.tagCount,
            "NoLimelightHubPose"
        );
    }

    static boolean isUsablePoseEstimate(LimelightHelpers.PoseEstimate poseEstimate) {
        return poseEstimate != null
            && poseEstimate.tagCount >= MIN_TAG_COUNT_TO_ACQUIRE
            && isFinitePose(poseEstimate.pose);
    }

    static double calculateTargetHeadingRad(Pose2d robotPoseField, Translation2d hubCenter) {
        return Math.atan2(
            hubCenter.getY() - robotPoseField.getY(),
            hubCenter.getX() - robotPoseField.getX()
        );
    }

    static double calculateDistanceMeters(Pose2d robotPoseField, Translation2d hubCenter) {
        return robotPoseField.getTranslation().getDistance(hubCenter);
    }

    static double calculateHeadingErrorRad(double currentHeadingRad, double targetHeadingRad) {
        return MathUtil.angleModulus(targetHeadingRad - currentHeadingRad);
    }

    private LimelightHelpers.PoseEstimate getAlliancePoseEstimate() {
        return FieldGeometry.isBlue()
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue(FRONT_LIMELIGHT)
            : LimelightHelpers.getBotPoseEstimate_wpiRed(FRONT_LIMELIGHT);
    }

    private void publishTelemetry(
        LimelightHelpers.PoseEstimate poseEstimate,
        PoseAimTarget poseAimTarget,
        double currentHeadingDeg,
        double targetHeadingDeg,
        double headingErrorDeg
    ) {
        if (!debugTelemetryEnabledSupplier.getAsBoolean()) {
            return;
        }

        Pose2d pose = poseEstimate == null ? null : poseEstimate.pose;
        double poseX = isFinitePose(pose) ? pose.getX() : Double.NaN;
        double poseY = isFinitePose(pose) ? pose.getY() : Double.NaN;
        int tagCount = poseEstimate == null ? 0 : poseEstimate.tagCount;

        SmartDashboard.putString("AutoShoot/AlignSource", poseAimTarget.alignSource());
        SmartDashboard.putNumber("AutoShoot/AlignTxDeg", headingErrorDeg);
        SmartDashboard.putNumber("AutoShoot/AlignErrorDeg", headingErrorDeg);
        SmartDashboard.putNumber("AutoShoot/AlignDistanceMeters", poseAimTarget.distanceMeters());
        SmartDashboard.putNumber("AutoShoot/AlignDirectVisibleTagCount", tagCount);
        SmartDashboard.putBoolean("AutoShoot/AlignDirectLatched", poseAimTarget.usingLatchedHeading());
        SmartDashboard.putNumber("AutoShoot/AlignPoseTagCount", tagCount);
        SmartDashboard.putNumber("AutoShoot/AlignPoseX", poseX);
        SmartDashboard.putNumber("AutoShoot/AlignPoseY", poseY);
        SmartDashboard.putNumber("AutoShoot/AlignTargetHeadingDeg", targetHeadingDeg);
        SmartDashboard.putNumber("AutoShoot/AlignCurrentHeadingDeg", currentHeadingDeg);
        SmartDashboard.putNumber("AutoShoot/AlignHeadingErrorDeg", headingErrorDeg);
    }

    private static boolean isFinitePose(Pose2d pose) {
        return pose != null
            && Double.isFinite(pose.getX())
            && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians());
    }
}
