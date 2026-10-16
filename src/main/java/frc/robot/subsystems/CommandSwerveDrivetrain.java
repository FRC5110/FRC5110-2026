package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.AllianceUtil;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;
    private Alliance m_lastAppliedPerspectiveAlliance = null;
    private boolean m_wasDisabled = false;

    // ================== LIMELIGHT SETUP ==================
    private final NetworkTable limelightTable =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.Front.LIMELIGHT_NAME);

    private final NetworkTableEntry botposeEntry =
        limelightTable.getEntry("botpose_wpiblue");

    private final NetworkTableEntry tvEntry =
        limelightTable.getEntry("tv");

    private final NetworkTableEntry tidEntry =
        limelightTable.getEntry("tid");

    private final NetworkTableEntry tlEntry =
        limelightTable.getEntry("tl");

    private final NetworkTableEntry clEntry =
        limelightTable.getEntry("cl");

    // ================== APRILTAG LATCHING ==================
    private Integer lastSeenAprilTag = null;
    private boolean visionOdometryEnabled = false;

    // ================== CONSTRUCTORS ==================
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency,
              odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // ================== VISION ODOMETRY UPDATE ==================
    private void updateVisionOdometry() {

        // tv == 1 means valid target detected
        if (tvEntry.getDouble(0) != 1) {
            return;
        }

        double[] poseArray = botposeEntry.getDoubleArray(new double[6]);

        if (poseArray.length < 6) {
            return;
        }

        if (!Double.isFinite(poseArray[0])
            || !Double.isFinite(poseArray[1])
            || !Double.isFinite(poseArray[5])) {
            return;
        }

        Pose2d visionPose = new Pose2d(
            poseArray[0], // X meters
            poseArray[1], // Y meters
            Rotation2d.fromDegrees(poseArray[5]) // rotation degrees
        );

        // total latency in seconds
        double latency =
            (tlEntry.getDouble(0) + clEntry.getDouble(0)) / 1000.0;

        double timestamp = Timer.getFPGATimestamp() - latency;

        addVisionMeasurement(visionPose, timestamp);
    }

    // ================== PERIODIC ==================
    @Override
    public void periodic() {
        boolean isDisabled = DriverStation.isDisabled();
        Alliance allianceColor = AllianceUtil.getAllianceOrBlue();

        if (!m_hasAppliedOperatorPerspective
                || allianceColor != m_lastAppliedPerspectiveAlliance
                || (isDisabled && !m_wasDisabled)) {
            setOperatorPerspectiveForward(
                allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
            m_lastAppliedPerspectiveAlliance = allianceColor;
        }
        m_wasDisabled = isDisabled;

        if (!isDisabled && tvEntry.getDouble(0.0) == 1.0) {
            double tid = tidEntry.getDouble(Double.NaN);
            if (Double.isFinite(tid)) {
                lastSeenAprilTag = (int) Math.round(tid);
            }
        }

        if (visionOdometryEnabled) {
            updateVisionOdometry();
        }
    }

    // ================== POSE ACCESS ==================
    public Pose2d getPose() {
        return getState().Pose;
    }

    public void stop() {
        setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    public void setVisionOdometryEnabled(boolean enabled) {
        visionOdometryEnabled = enabled;
    }

    public boolean isVisionOdometryEnabled() {
        return visionOdometryEnabled;
    }

    public boolean resetHeadingFromVision() {
        if (tvEntry.getDouble(0) != 1) {
            return false;
        }

        double[] poseArray = botposeEntry.getDoubleArray(new double[6]);
        if (poseArray.length < 6 || !Double.isFinite(poseArray[5])) {
            return false;
        }

        Pose2d currentPose = getPose();
        Pose2d resetPose = new Pose2d(
            currentPose.getTranslation(),
            Rotation2d.fromDegrees(poseArray[5])
        );
        resetPose(resetPose);
        return true;
    }

    public void setLastSeenAprilTag(int id) {
        lastSeenAprilTag = id;
    }

    public Optional<Integer> getLastSeenAprilTag() {
        return Optional.ofNullable(lastSeenAprilTag);
    }

    // ================== SIM SUPPORT ==================
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds)
        );
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        );
    }
}
