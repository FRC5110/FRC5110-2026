package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    public enum TelemetryProfile {
        MATCH,
        PIT,
        DEBUG;

        public boolean includesDriveHealth() {
            return this != MATCH;
        }

        public boolean includesDriveDebug() {
            return this == DEBUG;
        }
    }

    private static final boolean CTRE_SIGNAL_LOGGER_HARD_DISABLED = true;
    private static final double FIELD_PUBLISH_SEC = 0.10;
    private static final double DRIVE_HEALTH_PUBLISH_SEC = 0.10;
    private static final double DRIVE_DEBUG_PUBLISH_SEC = 0.10;
    private final double MaxSpeed;
    private boolean ctreSignalLoggerStarted = false;
    private int signalLogLoopCounter = 0;
    private TelemetryProfile activeProfile = TelemetryProfile.MATCH;
    private double lastFieldPublishSec = Double.NEGATIVE_INFINITY;
    private double lastDriveHealthPublishSec = Double.NEGATIVE_INFINITY;
    private double lastDriveDebugPublishSec = Double.NEGATIVE_INFINITY;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;

        if (CTRE_SIGNAL_LOGGER_HARD_DISABLED) {
            // Overwrite any previously latched dashboard value so logging stays off.
            SmartDashboard.putBoolean(Constants.Logging.ENABLE_CTRE_SIGNAL_LOGGER_KEY, false);
        } else {
            SmartDashboard.setDefaultBoolean(
                Constants.Logging.ENABLE_CTRE_SIGNAL_LOGGER_KEY,
                Constants.Logging.ENABLE_CTRE_SIGNAL_LOGGER_DEFAULT
            );
        }
        SmartDashboard.setDefaultNumber(
            Constants.Logging.CTRE_SIGNAL_LOG_EVERY_N_LOOPS_KEY,
            Constants.Logging.CTRE_SIGNAL_LOG_EVERY_N_LOOPS_DEFAULT
        );
        maybeStartSignalLogger();

        /* Module Mechanism2d dashboard widgets disabled to reduce loop overhead. */
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    public void setProfile(TelemetryProfile profile) {
        activeProfile = profile == null ? TelemetryProfile.MATCH : profile;
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        maybeStartSignalLogger();

        double nowSec = Timer.getFPGATimestamp();
        boolean publishField = shouldPublish(nowSec, lastFieldPublishSec, FIELD_PUBLISH_SEC);
        boolean publishDriveHealth = activeProfile.includesDriveHealth()
            && shouldPublish(nowSec, lastDriveHealthPublishSec, DRIVE_HEALTH_PUBLISH_SEC);
        boolean publishDriveDebug = activeProfile.includesDriveDebug()
            && shouldPublish(nowSec, lastDriveDebugPublishSec, DRIVE_DEBUG_PUBLISH_SEC);

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        if (publishDriveHealth || publishDriveDebug) {
            drivePose.set(state.Pose);
            driveSpeeds.set(state.Speeds);
            driveTimestamp.set(state.Timestamp);
            driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
            lastDriveHealthPublishSec = nowSec;
        }

        if (publishDriveDebug) {
            driveModuleStates.set(state.ModuleStates);
            driveModuleTargets.set(state.ModuleTargets);
            driveModulePositions.set(state.ModulePositions);
            lastDriveDebugPublishSec = nowSec;
        }

        boolean signalLoggingEnabled = SmartDashboard.getBoolean(
            Constants.Logging.ENABLE_CTRE_SIGNAL_LOGGER_KEY,
            Constants.Logging.ENABLE_CTRE_SIGNAL_LOGGER_DEFAULT
        );
        int signalLogEveryNLoops = Math.max(
            1,
            (int) Math.round(SmartDashboard.getNumber(
                Constants.Logging.CTRE_SIGNAL_LOG_EVERY_N_LOOPS_KEY,
                Constants.Logging.CTRE_SIGNAL_LOG_EVERY_N_LOOPS_DEFAULT
            ))
        );

        if (ctreSignalLoggerStarted && signalLoggingEnabled) {
            signalLogLoopCounter++;
            if (signalLogLoopCounter >= signalLogEveryNLoops) {
                SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
                SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
                SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
                SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
                signalLogLoopCounter = 0;
            }
        }

        if (publishField) {
            fieldTypePub.set("Field2d");
            fieldPub.set(m_poseArray);
            lastFieldPublishSec = nowSec;
        }

        if (publishDriveDebug) {
            for (int i = 0; i < 4; ++i) {
                m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
                m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
                m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
            }
        }
    }

    private void maybeStartSignalLogger() {
        if (CTRE_SIGNAL_LOGGER_HARD_DISABLED) {
            return;
        }

        if (ctreSignalLoggerStarted) {
            return;
        }

        boolean signalLoggingEnabled = SmartDashboard.getBoolean(
            Constants.Logging.ENABLE_CTRE_SIGNAL_LOGGER_KEY,
            Constants.Logging.ENABLE_CTRE_SIGNAL_LOGGER_DEFAULT
        );
        if (signalLoggingEnabled) {
            SignalLogger.start();
            ctreSignalLoggerStarted = true;
            signalLogLoopCounter = 0;
        }
    }

    private static boolean shouldPublish(double nowSec, double lastPublishSec, double intervalSec) {
        return !Double.isFinite(lastPublishSec) || (nowSec - lastPublishSec) >= intervalSec;
    }
}
