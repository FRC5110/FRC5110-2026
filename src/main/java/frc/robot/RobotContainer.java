package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.AutoAlignFieldCentric;
import frc.robot.commands.CalibrateHubDistanceCommand;
import frc.robot.commands.CalibrateShooterDistanceCommand;
import frc.robot.commands.AlignToFuelHubCommand;
import frc.robot.commands.FeedConveyorWhenShooterReadyCommand;
import frc.robot.commands.ManualConveyorArmCommand;
import frc.robot.commands.MoveConveyorArmToAbsolutePositionCommand;
import frc.robot.commands.RunConveyorAtRpmCommand;
import frc.robot.commands.RunConveyorPercentCommand;
import frc.robot.commands.RunIntakeRollerAtRpmCommand;
import frc.robot.commands.auton.AutoShootFeedWhenReadyCommand;
import frc.robot.commands.auton.RunConveyorForwardAutoCommand;
import frc.robot.commands.auton.RunConveyorReverseAutoCommand;
import frc.robot.commands.auton.RunIntakeForwardAutoCommand;
import frc.robot.commands.auton.RunIntakeReverseAutoCommand;
import frc.robot.commands.ShootAndFeedWhenReady;
import frc.robot.commands.AutoGetDistanceShootFeedCommand;
import frc.robot.commands.SpinShooterAtRpmCommand;
import frc.robot.commands.auton.TimedShootAndFeedWhenReadyCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BlinkinLedSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class RobotContainer {

    private static final double HUB_TAG_DISTANCE_LATCH_SEC = 0.10;
    private static final double HUB_TAG_DISTANCE_SAMPLE_CACHE_SEC = 0.02;
    private static final double HUB_DISTANCE_DEBUG_PUBLISH_SEC = 0.20;
    private static final String PROFILE_MATCH = "Match";
    private static final String PROFILE_PIT = "Pit";
    private static final String PROFILE_DEBUG = "Debug";
    private static final String ELIGIBLE_MATCH_ONLY = PROFILE_MATCH;
    private static final String ELIGIBLE_ALL_PROFILES = PROFILE_MATCH + "," + PROFILE_PIT + "," + PROFILE_DEBUG;

    /* ================= CONTROLLER ================= */
    private final CommandXboxController driverController =
            new CommandXboxController(0);
    private final CommandXboxController auxController =
            new CommandXboxController(1);

    /* ================= DRIVETRAIN ================= */
    public final CommandSwerveDrivetrain drivetrain =
            TunerConstants.createDrivetrain();
    private final ConveyorArmSubsystem conveyorArm =
            new ConveyorArmSubsystem();
    private final IntakeRollerSubsystem intakeRoller =
            new IntakeRollerSubsystem();
    private final ShooterFlywheelSubsystem shooterFlywheel =
            new ShooterFlywheelSubsystem();
    private final ConveyorSubsystem conveyor =
            new ConveyorSubsystem();
    private final BlinkinLedSubsystem ledSubsystem =
            new BlinkinLedSubsystem();
    private final Telemetry telemetry =
            new Telemetry(Constants.RobotContainer.MAX_SPEED);
    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTableEntry profileRequestedEntry =
            networkTables.getTable("Dashboard").getEntry("ProfileRequested");
    private final NetworkTableEntry profileActiveEntry =
            networkTables.getTable("Dashboard").getEntry("ProfileActive");
    private final NetworkTableEntry profileEligibleEntry =
            networkTables.getTable("Dashboard").getEntry("ProfileEligible");
    private final NetworkTableEntry tuningEligibleEntry =
            networkTables.getTable("Dashboard").getEntry("TuningEligible");
    private final NetworkTableEntry tuningArmedEntry =
            networkTables.getTable("Dashboard").getEntry("TuningArmed");
    private final NetworkTableEntry debugEnabledEntry =
            networkTables.getTable("Dashboard").getEntry("DebugEnabled");
    private final NetworkTableEntry tuningApplyRequestedEntry =
            networkTables.getTable("Dashboard").getEntry("TuningApplyRequested");
    private final NetworkTableEntry tuningStatusEntry =
            networkTables.getTable("Dashboard").getEntry("TuningStatus");
    private final NetworkTableEntry shooterCalibrationTargetRpmEntry =
            networkTables.getTable("Tuning/Calibration").getEntry("ShooterTargetRPM");
    private final NetworkTableEntry shooterCalibrationFeedEnabledEntry =
            networkTables.getTable("Tuning/Calibration").getEntry("ShooterAutoFeedEnabled");
    private final NetworkTableEntry shooterDistanceRpmTrimDraftEntry =
            networkTables.getTable("Tuning/Draft/Shooter").getEntry("DistanceRPMTrim");
    private final NetworkTableEntry shooterDistanceRpmTrimActiveEntry =
            networkTables.getTable("Tuning/Active/Shooter").getEntry("DistanceRPMTrim");

    /* ================= DRIVE REQUEST ================= */
    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle povFacingRequest =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withMaxAbsRotationalRate(Constants.RobotContainer.MAX_ROT);
    private final SwerveRequest.ApplyRobotSpeeds autoRequest =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity);
    private final SendableChooser<Command> autonomousChooser;
    private String lastFieldResetSource = "None";
    private double lastGoodHubTagDistanceMeters = Double.NaN;
    private double lastGoodHubTagDistanceTimestampSec = Double.NEGATIVE_INFINITY;
    private double lastHubDistanceDebugPublishTimestampSec = Double.NEGATIVE_INFINITY;
    private HubTagVision.HubCenterEstimate cachedHubCenterEstimate = HubTagVision.HubCenterEstimate.EMPTY;
    private double cachedHubTagDistanceTimestampSec = Double.NEGATIVE_INFINITY;
    private boolean limelightJsonWarm = false;
    private String limelightJsonWarmState = "Pending";
    private Telemetry.TelemetryProfile activeTelemetryProfile = Telemetry.TelemetryProfile.MATCH;
    private boolean tuningArmed = false;
    private boolean debugTelemetryEnabled = false;
    private double activeShooterDistanceRpmTrim = Constants.ShooterTune.DISTANCE_RPM_TRIM_DEFAULT;
    private boolean lastTuningApplyRequested = false;

    public RobotContainer() {

        povFacingRequest.HeadingController.setPID(5.0, 0.0, 0.1);
        povFacingRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        povFacingRequest.HeadingController.setTolerance(
            Math.toRadians(Constants.RobotContainer.HEADING_HOLD_TOLERANCE_DEG));
        initializeDashboardControls();
        conveyorArm.setTelemetryEnabledSuppliers(this::isPitProfileActive, this::isDebugProfileActive);
        shooterFlywheel.setTelemetryEnabledSuppliers(this::isPitProfileActive, this::isDebugProfileActive);
        shooterFlywheel.setDistanceRpmTrimSupplier(() -> activeShooterDistanceRpmTrim);

        registerPathPlannerNamedCommands();
        autonomousChooser = configurePathPlannerAutonomous();
        configureBindings();
        requestFrontLimelightAprilTagPipeline();
        publishLimelightJsonWarmStatus();
        refreshDashboardControlState();
    }

    private void initializeDashboardControls() {
        setEntryStringIfChanged(profileRequestedEntry, PROFILE_MATCH);
        setEntryStringIfChanged(profileActiveEntry, PROFILE_MATCH);
        setEntryStringIfChanged(profileEligibleEntry, ELIGIBLE_MATCH_ONLY);
        setEntryBooleanIfChanged(tuningEligibleEntry, false);
        setEntryBooleanIfChanged(tuningArmedEntry, false);
        setEntryBooleanIfChanged(debugEnabledEntry, false);
        setEntryBooleanIfChanged(tuningApplyRequestedEntry, false);
        setEntryStringIfChanged(tuningStatusEntry, "MatchLocked");

        setEntryDoubleIfChanged(shooterCalibrationTargetRpmEntry, Constants.RobotContainer.TEST_SHOOT_RPM);
        setEntryBooleanIfChanged(shooterCalibrationFeedEnabledEntry, false);
        activeShooterDistanceRpmTrim = Constants.ShooterTune.DISTANCE_RPM_TRIM_DEFAULT;
        setEntryDoubleIfChanged(shooterDistanceRpmTrimDraftEntry, activeShooterDistanceRpmTrim);
        setEntryDoubleIfChanged(shooterDistanceRpmTrimActiveEntry, activeShooterDistanceRpmTrim);

        telemetry.setProfile(activeTelemetryProfile);
        SmartDashboard.putString("Drive/FieldResetSource", lastFieldResetSource);
        SmartDashboard.putString("AutoShoot/CalibrationState", "Idle");
        SmartDashboard.putString("AutoShoot/ShooterCalibrationState", "Idle");
        SmartDashboard.putNumber("AutoShoot/ShooterCalibrationLatchedDistanceMeters", Double.NaN);
        SmartDashboard.putString("AutoShoot/ShooterCalibrationLatchedDistanceSource", "Unknown");
        SmartDashboard.putNumber("AutoShoot/ShooterCalibrationSuggestedRPM", 0.0);
        SmartDashboard.putNumber("AutoShoot/ShooterCalibrationCurrentTargetRPM", 0.0);
        SmartDashboard.putNumber("AutoShoot/ShooterCalibrationCurrentRPM", 0.0);
        SmartDashboard.putBoolean("AutoShoot/ShooterCalibrationAtSpeed", false);
        SmartDashboard.putBoolean("AutoShoot/ShooterCalibrationCanFeed", false);
        SmartDashboard.putBoolean("AutoShoot/ShooterCalibrationUsingBoost", false);
        SmartDashboard.putBoolean("AutoShoot/ShooterCalibrationFeedEnabled", false);
        SmartDashboard.putBoolean("AutoShoot/ShooterCalibrationForceFeed", false);
        SmartDashboard.putBoolean("AutoShoot/ShooterCalibrationReadyToFeed", false);
        SmartDashboard.putNumber(Constants.ShooterTune.DISTANCE_RPM_TRIM_KEY, activeShooterDistanceRpmTrim);
    }

    public void forceMatchTelemetryProfile() {
        setEntryStringIfChanged(profileRequestedEntry, PROFILE_MATCH);
        setEntryBooleanIfChanged(debugEnabledEntry, false);
        setEntryBooleanIfChanged(tuningArmedEntry, false);
        setEntryBooleanIfChanged(tuningApplyRequestedEntry, false);
        refreshDashboardControlState();
    }

    private void refreshDashboardControlState() {
        boolean pitDebugEligible = isPitDebugProfileEligible();
        Telemetry.TelemetryProfile requestedProfile =
            parseRequestedTelemetryProfile(profileRequestedEntry.getString(PROFILE_MATCH));
        boolean requestedDebugEnabled = debugEnabledEntry.getBoolean(false);
        boolean requestedTuningArmed = tuningArmedEntry.getBoolean(false);

        if (!pitDebugEligible) {
            requestedProfile = Telemetry.TelemetryProfile.MATCH;
            requestedDebugEnabled = false;
            requestedTuningArmed = false;
            setEntryStringIfChanged(profileRequestedEntry, PROFILE_MATCH);
            setEntryBooleanIfChanged(debugEnabledEntry, false);
            setEntryBooleanIfChanged(tuningArmedEntry, false);
            setEntryBooleanIfChanged(tuningApplyRequestedEntry, false);
        }

        Telemetry.TelemetryProfile resolvedProfile = switch (requestedProfile) {
            case MATCH -> Telemetry.TelemetryProfile.MATCH;
            case PIT -> pitDebugEligible ? Telemetry.TelemetryProfile.PIT : Telemetry.TelemetryProfile.MATCH;
            case DEBUG -> pitDebugEligible && requestedDebugEnabled
                ? Telemetry.TelemetryProfile.DEBUG
                : pitDebugEligible
                    ? Telemetry.TelemetryProfile.PIT
                    : Telemetry.TelemetryProfile.MATCH;
        };

        activeTelemetryProfile = resolvedProfile;
        debugTelemetryEnabled = resolvedProfile == Telemetry.TelemetryProfile.DEBUG;
        tuningArmed = resolvedProfile == Telemetry.TelemetryProfile.PIT && requestedTuningArmed;

        telemetry.setProfile(activeTelemetryProfile);

        if (!tuningArmed) {
            setEntryBooleanIfChanged(tuningArmedEntry, false);
        }

        String applyStatus = processTuningApplyRequest();
        setEntryStringIfChanged(profileActiveEntry, profileToDashboardString(activeTelemetryProfile));
        setEntryStringIfChanged(profileEligibleEntry, pitDebugEligible ? ELIGIBLE_ALL_PROFILES : ELIGIBLE_MATCH_ONLY);
        setEntryBooleanIfChanged(tuningEligibleEntry, isPitProfileActive());
        setEntryBooleanIfChanged(debugEnabledEntry, debugTelemetryEnabled || requestedDebugEnabled);
        setEntryDoubleIfChanged(shooterDistanceRpmTrimActiveEntry, activeShooterDistanceRpmTrim);
        updateTuningStatus(applyStatus);
    }

    private String processTuningApplyRequest() {
        boolean applyRequested = tuningApplyRequestedEntry.getBoolean(false);
        if (!applyRequested) {
            lastTuningApplyRequested = false;
            return null;
        }

        if (lastTuningApplyRequested) {
            return null;
        }

        lastTuningApplyRequested = true;
        setEntryBooleanIfChanged(tuningApplyRequestedEntry, false);

        if (!isTuningControlEnabled()) {
            return "ApplyIgnored";
        }

        applyDashboardTuningValues();
        return "Applied";
    }

    private void applyDashboardTuningValues() {
        double requestedTrimRpm =
            shooterDistanceRpmTrimDraftEntry.getDouble(Constants.ShooterTune.DISTANCE_RPM_TRIM_DEFAULT);
        activeShooterDistanceRpmTrim = Double.isFinite(requestedTrimRpm)
            ? requestedTrimRpm
            : Constants.ShooterTune.DISTANCE_RPM_TRIM_DEFAULT;
        setEntryDoubleIfChanged(shooterDistanceRpmTrimActiveEntry, activeShooterDistanceRpmTrim);
        setEntryDoubleIfChanged(shooterDistanceRpmTrimDraftEntry, activeShooterDistanceRpmTrim);
        SmartDashboard.putNumber(Constants.ShooterTune.DISTANCE_RPM_TRIM_KEY, activeShooterDistanceRpmTrim);
    }

    private void updateTuningStatus(String applyStatus) {
        if (applyStatus != null) {
            setEntryStringIfChanged(tuningStatusEntry, applyStatus);
            return;
        }

        if (activeTelemetryProfile == Telemetry.TelemetryProfile.DEBUG) {
            setEntryStringIfChanged(tuningStatusEntry, "DebugEnabled");
            return;
        }

        if (!isPitDebugProfileEligible()) {
            setEntryStringIfChanged(tuningStatusEntry, "MatchLocked");
            return;
        }

        if (!isPitProfileActive()) {
            setEntryStringIfChanged(tuningStatusEntry, "MatchActive");
            return;
        }

        if (!tuningArmed) {
            setEntryStringIfChanged(tuningStatusEntry, "PitReady");
            return;
        }

        boolean draftPending = differs(
                shooterDistanceRpmTrimDraftEntry.getDouble(activeShooterDistanceRpmTrim),
                activeShooterDistanceRpmTrim
            );
        setEntryStringIfChanged(tuningStatusEntry, draftPending ? "DraftPending" : "TuningArmed");
    }

    private boolean isPitDebugProfileEligible() {
        return !DriverStation.isFMSAttached() && (DriverStation.isDisabled() || DriverStation.isTestEnabled());
    }

    private Telemetry.TelemetryProfile parseRequestedTelemetryProfile(String rawProfile) {
        if (PROFILE_PIT.equalsIgnoreCase(rawProfile)) {
            return Telemetry.TelemetryProfile.PIT;
        }
        if (PROFILE_DEBUG.equalsIgnoreCase(rawProfile)) {
            return Telemetry.TelemetryProfile.DEBUG;
        }
        return Telemetry.TelemetryProfile.MATCH;
    }

    private String profileToDashboardString(Telemetry.TelemetryProfile profile) {
        return switch (profile) {
            case MATCH -> PROFILE_MATCH;
            case PIT -> PROFILE_PIT;
            case DEBUG -> PROFILE_DEBUG;
        };
    }

    private boolean isPitProfileActive() {
        return activeTelemetryProfile == Telemetry.TelemetryProfile.PIT;
    }

    private boolean isDebugProfileActive() {
        return activeTelemetryProfile == Telemetry.TelemetryProfile.DEBUG;
    }

    private boolean isTuningControlEnabled() {
        return isPitProfileActive() && tuningArmed;
    }

    private boolean differs(double first, double second) {
        return !Double.isFinite(first)
            || !Double.isFinite(second)
            || Math.abs(first - second) > 1e-6;
    }

    private static void setEntryBooleanIfChanged(NetworkTableEntry entry, boolean value) {
        if (entry.getBoolean(!value) != value) {
            entry.setBoolean(value);
        }
    }

    private static void setEntryDoubleIfChanged(NetworkTableEntry entry, double value) {
        if (!Double.isFinite(entry.getDouble(Double.NaN)) || Math.abs(entry.getDouble(Double.NaN) - value) > 1e-9) {
            entry.setDouble(value);
        }
    }

    private static void setEntryStringIfChanged(NetworkTableEntry entry, String value) {
        if (!value.equals(entry.getString(""))) {
            entry.setString(value);
        }
    }

    private void configureBindings() {
        /* ================= CONVEYOR ARM MANUAL (AUX RIGHT STICK Y) ================= */
        conveyorArm.setDefaultCommand(
                new ManualConveyorArmCommand(
                        conveyorArm,
                        auxController::getRightY,
                        Constants.RobotContainer.DEADBAND,
                        conveyorArm::getManualSpeedScale
                )
        );

        /* ================= CONVEYOR ARM ENDPOINT PRESETS (AUX Y/A) ================= */
        auxController.y().onTrue(
                new MoveConveyorArmToAbsolutePositionCommand(
                        conveyorArm,
                        Constants.ConveyorArm.ABSOLUTE_ENCODER_HIGH_LIMIT_DEGREES,
                        "high"
                )
        );
        auxController.a().onTrue(
                new MoveConveyorArmToAbsolutePositionCommand(
                        conveyorArm,
                        Constants.ConveyorArm.ABSOLUTE_ENCODER_LOW_LIMIT_DEGREES,
                        "low"
                )
        );

        // Temporary climber down-only default on AUX right stick kept for reference:
        // climber.setDefaultCommand(
        //         new RunClimberDownFromAuxStickCommand(
        //                 climber,
        //                 auxController::getRightY,
        //                 Constants.RobotContainer.DEADBAND,
        //                 Constants.RobotContainer.CLIMBER_MAX_OUTPUT
        //         )
        // );

        /* ================= DEFAULT DRIVE ================= */
        drivetrain.setDefaultCommand(
                drivetrain.run(() -> {

                    double vx = applyDeadband(driverController.getLeftY())
                            * Constants.RobotContainer.DRIVER_TRANSLATION_INVERT
                            * Constants.RobotContainer.MAX_SPEED;
                    double vy = applyDeadband(driverController.getLeftX())
                            * Constants.RobotContainer.DRIVER_TRANSLATION_INVERT
                            * Constants.RobotContainer.MAX_SPEED;
                    double omega = applyDeadband(driverController.getRightX())
                            * Constants.RobotContainer.DRIVER_ROTATION_INVERT
                            * Constants.RobotContainer.MAX_ROT;

                    drivetrain.setControl(
                            driveRequest
                                    .withVelocityX(vx)
                                    .withVelocityY(vy)
                                    .withRotationalRate(omega)
                    );
                })
        );

        /* ================= FIELD ORIENT RESET (Press A) ================= */
        driverController.a().onTrue(
                Commands.runOnce(this::resetFieldOrientationWithVision, drivetrain)
        );

        // Old driver orbit binding kept for reference:
        // driverController.b().whileTrue(
        //         Commands.startEnd(
        //                 () -> {
        //                     orbitMode = true;
        //                     headingPID.reset();
        //                 },
        //                 () -> orbitMode = false
        //         )
        // );
        /* ================= SNAP (POV UP/DOWN) ================= */
        driverController.povUp().whileTrue(createDriverFacingHoldCommand(0.0));
        driverController.povDown().whileTrue(createDriverFacingHoldCommand(180.0));

        /* ================= CLIMBER LATCH MANUAL (DRIVER POV RIGHT/LEFT) ================= */
        // driverController.povRight().onTrue(
        //         new NudgeClimberLatchCommand(climber, false)
        // );
        // driverController.povLeft().onTrue(
        //         new NudgeClimberLatchCommand(climber, true)
        // );

        /* ================= ALIGN TO FUEL HUB (Press X) ================= */
        driverController.x().whileTrue(
                new AlignToFuelHubCommand(
                        drivetrain,
                        ignored -> {},
                        () -> {},
                        () -> {},
                        this::isDebugProfileActive
                ).withTimeout(Constants.RobotContainer.FUEL_HUB_ALIGN_TIMEOUT_SEC)
        );

        /* ================= FIELD ORIENT RESET (Press Y, No Vision) ================= */
        driverController.y().onTrue(
                Commands.runOnce(this::resetFieldOrientationWithoutVision, drivetrain)
        );

        /* ================= SHOOTER SYSID (TEST MODE ONLY) ================= */
        Trigger testMode = new Trigger(DriverStation::isTestEnabled);
        testMode.and(driverController.b()).whileTrue(
                shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        testMode.and(driverController.leftBumper()).whileTrue(
                shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        testMode.and(driverController.rightBumper()).whileTrue(
                shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        testMode.and(driverController.start()).whileTrue(
                shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        /* ================= CLIMBER DRIVE CONTROL (DRIVER RT/LT) ================= */
        // driverController.rightTrigger(Constants.RobotContainer.TRIGGER_DEADBAND)
        //         .or(driverController.leftTrigger(Constants.RobotContainer.TRIGGER_DEADBAND))
        //         .whileTrue(
        //                 new RunClimberFromTriggersCommand(
        //                         climber,
        //                         driverController::getRightTriggerAxis,
        //                         driverController::getLeftTriggerAxis,
        //                         Constants.RobotContainer.CLIMBER_MAX_OUTPUT,
        //                         Constants.RobotContainer.TRIGGER_DEADBAND
        //                 )
        //         );

        /* ================= CLIMBER LATCH (DRIVER BUMPERS) ================= */
        // driverController.leftBumper().onTrue(
        //         new SetClimberLatchCommand(climber, false)
        // );
        // driverController.rightBumper().onTrue(
        //         new SetClimberLatchCommand(climber, true)
        // );

        /* ================= FIXED RPM INTAKE (AUX TRIGGERS) ================= */
        auxController.rightTrigger(Constants.RobotContainer.TRIGGER_DEADBAND).whileTrue(
                new RunIntakeRollerAtRpmCommand(intakeRoller, 5000.0)
        );

        auxController.leftTrigger(Constants.RobotContainer.TRIGGER_DEADBAND).whileTrue(
                new RunIntakeRollerAtRpmCommand(intakeRoller, -3000.0)
        );

        /* ================= MANUAL SHOOT/FEED (AUX BUMPERS) ================= */
        // Old coordinated shoot/feed command kept for reference:
        // auxController.rightBumper().whileTrue(
        //         new ShootAndFeedWhenReady(
        //                 shooterFlywheel,
        //                 conveyor,
        //                 this::getShootCommandRPM
        //         )
        // );

        // Old coordinated shoot+feed binding kept for reference:
        // auxController.rightBumper().whileTrue(
        //         new AutoGetDistanceShootFeedCommand(
        //                 shooterFlywheel,
        //                 conveyor,
        //                 this::getDistanceBasedShootRPM
        //         )
        // );

        auxController.rightBumper().whileTrue(
                new SpinShooterAtRpmCommand(
                        shooterFlywheel,
                        this::getDistanceBasedShootRPM,
                        this::updateHubTagLedFeedback,
                        this::clearHubTagLedFeedback
                )
        );

        auxController.leftBumper().whileTrue(
                new FeedConveyorWhenShooterReadyCommand(
                        conveyor,
                        shooterFlywheel
                )
        );

        auxController.back().whileTrue(
                buildAuxBackDistanceShootFeedCommand()
        );

        new Trigger(() -> auxController.leftStick().getAsBoolean() && isTuningControlEnabled()).whileTrue(
                new CalibrateShooterDistanceCommand(
                        shooterFlywheel,
                        conveyor,
                        this::getDistanceToFuelHubMeters,
                        this::getLatestHubDistanceSource,
                        this::getShooterCalibrationTargetRPM,
                        this::isShooterCalibrationFeedEnabled,
                        auxController.rightStick()::getAsBoolean,
                        this::updateHubTagLedFeedback,
                        this::clearHubTagLedFeedback
                )
        );

        auxController.x().whileTrue(
                new RunConveyorAtRpmCommand(conveyor, 3000.0)
        );


        new Trigger(() -> auxController.b().getAsBoolean() && isTuningControlEnabled()).whileTrue(
                new CalibrateHubDistanceCommand(
                        this::updateHubDistanceCalibrationTelemetry,
                        this::clearHubTagLedFeedback
                )
        );

        auxController.povRight().whileTrue(
                new RunConveyorPercentCommand(
                        conveyor,
                        Constants.RobotContainer.SHOOT_CONVEYOR_PERCENT
                )
        );

        auxController.povLeft().whileTrue(
                new RunConveyorPercentCommand(
                        conveyor,
                        -Constants.RobotContainer.SHOOT_CONVEYOR_PERCENT
                )
        );

        /* ================= CLIMBER TO TOP LOCK (AUX X) ================= */
        // auxController.x().onTrue(
        //         new ClimbToTopLatchCommand(climber)
        // );

        /* ================= SPIN SHOOTER ONLY (AUX START) ================= */
        // Old AUX climb-distance bind kept for reference:
        // auxController.start().onTrue(
        //         new DriveToClimbDistanceCommand(
        //                 drivetrain,
        //                 climbVision,
        //                 Constants.Vision.Climb.TARGET_DISTANCE_METERS
        //         )
        // );
        auxController.start().whileTrue(
                new SpinShooterAtRpmCommand(
                        shooterFlywheel,
                        this::getPresetShotRPM
                )
        );

        // Old trigger shooter bindings (commented out so triggers can control intake):
        // auxController.leftTrigger(0.25).whileTrue(
        //         Commands.sequence(
        //                 new AlignToFuelHubCommand(drivetrain)
        //                         .withTimeout(FUEL_HUB_ALIGN_TIMEOUT_SEC),
        //                 new ShootFromDistanceCommand(
        //                         shooterFlywheel,
        //                         conveyor,
        //                         this::getDistanceToFuelHubMeters,
        //                         SHOOT_CONVEYOR_PERCENT
        //                 )
        //         )
        // );
        //
        // auxController.rightTrigger(0.25).whileTrue(
        //         Commands.startEnd(
        //                 () -> shooterFlywheel.setTargetRPM(TEST_SHOOT_RPM),
        //                 shooterFlywheel::stop,
        //                 shooterFlywheel
        //         )
        // );
    }

    private Command createDriverFacingHoldCommand(double degreesFromDriverForward) {
        return Commands.startRun(
                () -> povFacingRequest.HeadingController.reset(),
                () -> {
                    double vx = applyDeadband(driverController.getLeftY())
                            * Constants.RobotContainer.DRIVER_TRANSLATION_INVERT
                            * Constants.RobotContainer.MAX_SPEED;
                    double vy = applyDeadband(driverController.getLeftX())
                            * Constants.RobotContainer.DRIVER_TRANSLATION_INVERT
                            * Constants.RobotContainer.MAX_SPEED;

                    drivetrain.setControl(
                            povFacingRequest
                                    .withVelocityX(vx)
                                    .withVelocityY(vy)
                                    .withTargetDirection(driverRelativeHeadingToRobotFieldHeading(
                                            Math.toRadians(degreesFromDriverForward)))
                    );
                },
                drivetrain
        );
    }

    // DEBUG_TELEOP_FRONT: Converts a driver-relative heading request into the
    // actual shooter-front field heading target used by the robot.
    private Rotation2d driverRelativeHeadingToRobotFieldHeading(double driverRelativeHeadingRad) {
        double operatorForwardRad = drivetrain.getOperatorForwardDirection().getRadians();
        double teleopFrontOffsetRad =
            Math.toRadians(Constants.RobotContainer.DRIVER_TELEOP_FRONT_OFFSET_DEG);

        return Rotation2d.fromRadians(
            operatorForwardRad + driverRelativeHeadingRad + teleopFrontOffsetRad
        );
    }

    private void resetFieldOrientationWithVision() {
        boolean usedVisionHeading = drivetrain.resetHeadingFromVision();
        if (!usedVisionHeading) {
            drivetrain.seedFieldCentric();
        }
        finishFieldOrientationReset(usedVisionHeading ? "LimelightHeading" : "GyroSeed");
    }

    private void resetFieldOrientationWithoutVision() {
        drivetrain.seedFieldCentric();
        finishFieldOrientationReset("GyroSeedManual");
    }

    private void finishFieldOrientationReset(String source) {
        lastFieldResetSource = source;
        SmartDashboard.putString("Drive/FieldResetSource", source);
    }

    /* ================= SUPPLIERS ================= */

    @SuppressWarnings("unused")
private DoubleSupplier driverX() {
        return () -> applyDeadband(driverController.getLeftY())
                * Constants.RobotContainer.DRIVER_TRANSLATION_INVERT
                * Constants.RobotContainer.MAX_SPEED;
    }

    @SuppressWarnings("unused")
private DoubleSupplier driverY() {
        return () -> applyDeadband(driverController.getLeftX())
                * Constants.RobotContainer.DRIVER_TRANSLATION_INVERT
                * Constants.RobotContainer.MAX_SPEED;
    }

    /* ================= HELPERS ================= */

    @SuppressWarnings("unused")
private Pose2d getAllianceOutpostPose() {
        return isBlueAlliance()
                ? Constants.RobotContainer.BLUE_OUTPOST
                : Constants.RobotContainer.RED_OUTPOST;
    }

    private boolean isBlueAlliance() {
        return AllianceUtil.isBlue();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < Constants.RobotContainer.DEADBAND ? 0.0 : value;
    }

    private double getDistanceToFuelHubMeters() {
        Translation2d hubCenter = FieldGeometry.getFuelHubTrueCenter();
        double odometryDistance = drivetrain.getPose().getTranslation().getDistance(hubCenter);
        double nowSec = Timer.getFPGATimestamp();

        HubTagVision.HubCenterEstimate directHubEstimate = getDirectHubCenterEstimate();
        boolean usingDirectHubTagDistance = directHubEstimate.isValid();
        boolean usingLatchedHubTagDistance = false;
        double selectedDistance = Double.NaN;
        String source = "NoLimelightDistance";

        if (usingDirectHubTagDistance) {
            selectedDistance = directHubEstimate.distanceMeters();
            source = directHubEstimate.visibleHubTagCount() >= 2
                ? "LimelightHubTagsRobotSpace"
                : "LimelightHubTagRobotSpace";
            lastGoodHubTagDistanceMeters = selectedDistance;
            lastGoodHubTagDistanceTimestampSec = nowSec;
        } else if (Double.isFinite(lastGoodHubTagDistanceMeters)
                && (nowSec - lastGoodHubTagDistanceTimestampSec) <= HUB_TAG_DISTANCE_LATCH_SEC) {
            selectedDistance = lastGoodHubTagDistanceMeters;
            usingLatchedHubTagDistance = true;
            source = "LimelightHubTagLatched";
        } else {
            lastGoodHubTagDistanceMeters = Double.NaN;
            lastGoodHubTagDistanceTimestampSec = Double.NEGATIVE_INFINITY;
        }

        double hubTagLatchAgeSec = Double.isFinite(lastGoodHubTagDistanceTimestampSec)
            ? Math.max(0.0, nowSec - lastGoodHubTagDistanceTimestampSec)
            : Double.NaN;
        double recommendedRpm = Double.isFinite(selectedDistance)
            ? shooterFlywheel.getRecommendedRPMForDistanceMeters(selectedDistance)
            : 0.0;

        publishHubDistanceDebug(
            nowSec,
            source,
            selectedDistance,
            recommendedRpm,
            odometryDistance,
            Double.NaN,
            "Disabled",
            directHubEstimate,
            usingLatchedHubTagDistance,
            hubTagLatchAgeSec,
            directHubEstimate.visibleHubTagCount(),
            directHubEstimate.averageVisibleTagDistanceMeters()
        );

        return selectedDistance;
    }

    void warmHubDistanceVisionIfNeeded() {
        requestFrontLimelightAprilTagPipeline();

        if (limelightJsonWarm) {
            publishLimelightJsonWarmStatus();
            return;
        }

        if (!LimelightHelpers.tryWarmLatestResultsJson(Constants.Vision.Front.LIMELIGHT_NAME)) {
            limelightJsonWarmState = "WaitingForJson";
            publishLimelightJsonWarmStatus();
            return;
        }

        limelightJsonWarm = true;
        limelightJsonWarmState = "Warm";
        cachedHubCenterEstimate = HubTagVision.HubCenterEstimate.EMPTY;
        cachedHubTagDistanceTimestampSec = Double.NEGATIVE_INFINITY;
        getDirectHubCenterEstimate();
        publishLimelightJsonWarmStatus();
    }

    private HubTagVision.HubCenterEstimate getDirectHubCenterEstimate() {
        double nowSec = Timer.getFPGATimestamp();
        if (Double.isFinite(cachedHubTagDistanceTimestampSec)
                && (nowSec - cachedHubTagDistanceTimestampSec) <= HUB_TAG_DISTANCE_SAMPLE_CACHE_SEC) {
            return cachedHubCenterEstimate;
        }

        cachedHubCenterEstimate =
            HubTagVision.estimatePreferredFuelHubCenterRobotSpaceRealtime(
                Constants.Vision.Front.LIMELIGHT_NAME,
                drivetrain.getPose()
            );
        cachedHubTagDistanceTimestampSec = nowSec;
        return cachedHubCenterEstimate;
    }

    private double getDistanceBasedShootRPM() {
        double distanceMeters = getDistanceToFuelHubMeters();
        if (!Double.isFinite(distanceMeters)) {
            return 0.0;
        }
        return shooterFlywheel.getRecommendedRPMForDistanceMeters(distanceMeters);
    }

    private void updateHubDistanceCalibrationTelemetry() {
        getDistanceBasedShootRPM();
        updateHubTagLedFeedback();
    }

    private void updateHubTagLedFeedback() {
        setHubTagLedFeedback(getDirectHubCenterEstimate().isValid());
    }

    private String getLatestHubDistanceSource() {
        return SmartDashboard.getString("AutoShoot/HubDistanceSource", "Unknown");
    }

    private void setHubTagLedFeedback(boolean hasValidHubTag) {
        ledSubsystem.setState(
            hasValidHubTag
                ? BlinkinLedSubsystem.LedState.HUB_TAG_FOUND
                : BlinkinLedSubsystem.LedState.HUB_TAG_MISSING
        );
    }

    private void clearHubTagLedFeedback() {
        ledSubsystem.setState(getDefaultLedStateForCurrentMode());
    }

    private void publishLimelightJsonWarmStatus() {
        publishFrontLimelightPipelineStatus();
        SmartDashboard.putBoolean("Vision/LimelightJsonWarm", limelightJsonWarm);
        SmartDashboard.putString("Vision/LimelightJsonWarmState", limelightJsonWarmState);
    }

    private void requestFrontLimelightAprilTagPipeline() {
        LimelightHelpers.setPipelineIndex(
            Constants.Vision.Front.LIMELIGHT_NAME,
            Constants.Vision.Front.APRILTAG_PIPELINE_INDEX
        );
    }

    private void publishFrontLimelightPipelineStatus() {
        SmartDashboard.putNumber(
            "Vision/FrontLimelightTargetPipelineIndex",
            Constants.Vision.Front.APRILTAG_PIPELINE_INDEX
        );
        SmartDashboard.putNumber(
            "Vision/FrontLimelightActivePipelineIndex",
            LimelightHelpers.getCurrentPipelineIndex(Constants.Vision.Front.LIMELIGHT_NAME)
        );
        SmartDashboard.putString(
            "Vision/FrontLimelightActivePipelineType",
            LimelightHelpers.getCurrentPipelineType(Constants.Vision.Front.LIMELIGHT_NAME)
        );
    }

    private BlinkinLedSubsystem.LedState getDefaultLedStateForCurrentMode() {
        if (DriverStation.isTestEnabled()) {
            return BlinkinLedSubsystem.LedState.TEST;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return BlinkinLedSubsystem.LedState.AUTONOMOUS;
        }
        if (DriverStation.isTeleopEnabled()) {
            return BlinkinLedSubsystem.LedState.TELEOP;
        }
        return BlinkinLedSubsystem.LedState.DISABLED;
    }

    private double getShooterCalibrationTargetRPM() {
        if (!isTuningControlEnabled()) {
            return 0.0;
        }
        return shooterCalibrationTargetRpmEntry.getDouble(Constants.RobotContainer.TEST_SHOOT_RPM);
    }

    private boolean isShooterCalibrationFeedEnabled() {
        if (!isTuningControlEnabled()) {
            return false;
        }
        return shooterCalibrationFeedEnabledEntry.getBoolean(false);
    }

    private void publishHubDistanceDebug(
        double nowSec,
        String source,
        double selectedDistance,
        double recommendedRpm,
        double odometryDistance,
        double poseFallbackDistance,
        String poseFallbackSource,
        HubTagVision.HubCenterEstimate directHubEstimate,
        boolean usingLatchedHubTagDistance,
        double hubTagLatchAgeSec,
        int tagCount,
        double avgTagDist
    ) {
        if (Double.isFinite(lastHubDistanceDebugPublishTimestampSec)
                && (nowSec - lastHubDistanceDebugPublishTimestampSec) < HUB_DISTANCE_DEBUG_PUBLISH_SEC) {
            return;
        }

        lastHubDistanceDebugPublishTimestampSec = nowSec;

        SmartDashboard.putString("AutoShoot/HubDistanceSource", source);
        SmartDashboard.putNumber("AutoShoot/HubDistanceMetersSelected", selectedDistance);
        SmartDashboard.putNumber("AutoShoot/RPMFromDistance", recommendedRpm);
        SmartDashboard.putNumber("AutoShoot/LimelightTagCount", tagCount);
        SmartDashboard.putNumber("AutoShoot/LimelightAvgTagDistMeters", avgTagDist);

        if (!isPitProfileActive() && !isDebugProfileActive()) {
            return;
        }

        SmartDashboard.putNumber("AutoShoot/HubDistanceMetersOdom", odometryDistance);
        SmartDashboard.putString("AutoShoot/HubDistancePoseFallbackSource", poseFallbackSource);
        SmartDashboard.putNumber("AutoShoot/HubDistanceMetersPoseFallback", poseFallbackDistance);
        SmartDashboard.putNumber(
            "AutoShoot/HubDistanceMetersDirectRaw",
            directHubEstimate.distanceMeters()
        );
        SmartDashboard.putNumber(
            "AutoShoot/HubDistanceMetersDirectAvgTag",
            directHubEstimate.averageVisibleTagDistanceMeters()
        );
        SmartDashboard.putNumber(
            "AutoShoot/HubDistanceDirectVisibleTagCount",
            directHubEstimate.visibleHubTagCount()
        );
        SmartDashboard.putBoolean("AutoShoot/HubDistanceDirectExactFromPair", false);
        SmartDashboard.putBoolean(
            "AutoShoot/HubDistanceDirectPoseBased",
            directHubEstimate.poseBasedEstimate()
        );
        SmartDashboard.putNumber("AutoShoot/HubDistanceRobotSpaceX", directHubEstimate.robotSpaceX());
        SmartDashboard.putNumber("AutoShoot/HubDistanceRobotSpaceY", directHubEstimate.robotSpaceY());
        SmartDashboard.putNumber("AutoShoot/HubDistanceRobotSpaceBearingDeg", directHubEstimate.centerErrorDeg());
        SmartDashboard.putNumber("AutoShoot/HubDistanceEstimateSpreadM", directHubEstimate.estimateSpreadMeters());
        SmartDashboard.putBoolean("AutoShoot/HubDistanceDirectLatched", usingLatchedHubTagDistance);
        SmartDashboard.putNumber("AutoShoot/HubDistanceDirectLatchAgeSec", hubTagLatchAgeSec);
    }

    private double getPresetShotRPM() {
        double presetRPM = Constants.ShooterTune.PRESET_SHOT_RPM_DEFAULT;
        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putNumber("AutoShoot/PresetShotRPM", presetRPM);
        return presetRPM;
    }

    private double getShootCommandRPM() {
        boolean useDistanceRPM = Constants.ShooterTune.USE_DISTANCE_RPM_DEFAULT;

        if (useDistanceRPM) {
            return getDistanceBasedShootRPM();
        }

        return getPresetShotRPM();
    }

    private void registerPathPlannerNamedCommands() {
        NamedCommands.registerCommand("NoOp", Commands.none());

        // Safe starter durations for event-marker use.
        NamedCommands.registerCommand(
            "AutoIntakeForward1s",
            new RunIntakeForwardAutoCommand(intakeRoller).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AutoIntakeReverse1s",
            new RunIntakeReverseAutoCommand(intakeRoller).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AutoTimedShootFromDistance2p5s",
            buildTimedAutoShootFromDistance(2.5)
        );
        NamedCommands.registerCommand(
            "AutoShootFeedDistance2p5s",
            buildTimedAutoShootFromDistance(2.5)
        );
        NamedCommands.registerCommand(
            "AutoShootFeedFixed2p5s",
            buildTimedAutoShootAtRpm(Constants.RobotContainer.TEST_SHOOT_RPM, 2.5)
        );
        NamedCommands.registerCommand(
            "AutoShootFeedWhenReady5s",
            buildAutoShootFeedWhenReady(5.0)
        );
        NamedCommands.registerCommand(
            "AutoShootFeedWhenReady",
            buildAutoShootFeedWhenReady(5.0)
        );
        NamedCommands.registerCommand(
            "AutoSpinShooterPreset5s",
            buildSpinShooterAtPresetRpmAuto(5.0)
        );
        NamedCommands.registerCommand(
            "SpinShooterPreset5s",
            buildSpinShooterAtPresetRpmAuto(5.0)
        );
        NamedCommands.registerCommand(
            "AutoDistShootFeedCommand",
            buildAutoDistShootFeedAuto(2.5)
        );
        NamedCommands.registerCommand(
            "AutoGetDistanceShootFeedCommand",
            buildAuxBackDistanceShootFeedCommand()
        );
        NamedCommands.registerCommand(
            "AuxBackDistanceShootFeedCommand",
            buildAuxBackDistanceShootFeedCommand()
        );
        NamedCommands.registerCommand(
            "AutoGetDistanceShootFeed2p5s",
            buildAutoDistShootFeedAuto(2.5)
        );
        NamedCommands.registerCommand(
            "AutoDistShootFeed2p5s",
            buildAutoDistShootFeedAuto(2.5)
        );
        NamedCommands.registerCommand(
            "ShootFeedNoAlign",
            buildAutoDistShootFeedAuto(2.5)
        );
        NamedCommands.registerCommand(
            "AutoShootFeedNoAlign2p5s",
            buildAutoDistShootFeedAuto(2.5)
        );
        NamedCommands.registerCommand(
            "AlignToFuelHub",
            new AlignToFuelHubCommand(
                drivetrain,
                this::setHubTagLedFeedback,
                this::clearHubTagLedFeedback,
                () -> {},
                this::isDebugProfileActive
            ).withTimeout(Constants.RobotContainer.FUEL_HUB_ALIGN_TIMEOUT_SEC)
        );
        NamedCommands.registerCommand(
            "AlignToFuelHubCommand",
            new AlignToFuelHubCommand(
                drivetrain,
                this::setHubTagLedFeedback,
                this::clearHubTagLedFeedback,
                () -> {},
                this::isDebugProfileActive
            ).withTimeout(Constants.RobotContainer.FUEL_HUB_ALIGN_TIMEOUT_SEC)
        );
        NamedCommands.registerCommand(
            "AutoAlignFuelHubFieldCentric",
            buildAutoAlignFieldCentric(AutoAlignFieldCentric.AlignMode.FUEL_HUB)
        );
        NamedCommands.registerCommand(
            "AutoAlignOutpostFieldCentric",
            buildAutoAlignFieldCentric(AutoAlignFieldCentric.AlignMode.OUTPOST)
        );
        NamedCommands.registerCommand(
            "AutoShootThreeLowerFeed5s",
            buildAutoShootFeedWhenReady(5.0)
        );
        NamedCommands.registerCommand(
            "ShootThreeThenLowerArmAndFeedCommand",
            buildAutoShootFeedWhenReady(5.0)
        );
        NamedCommands.registerCommand(
            "AutoConveyorForward1s",
            new RunConveyorForwardAutoCommand(conveyor).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "RunConveyorForwardAutoCommand",
            new RunConveyorForwardAutoCommand(conveyor).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AutoConveyorReverse1s",
            new RunConveyorReverseAutoCommand(conveyor).withTimeout(1.0)
        );
        // Climber was removed. Keep legacy PathPlanner event names as no-ops so
        // existing autos still load without editing every .auto file immediately.
        NamedCommands.registerCommand(
            "AutoNudgeLatchDown",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoNudgeLatchUp",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoLatchDisengage",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoLatchEngage",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoIntakeRollerForward1s",
            new RunIntakeRollerAtRpmCommand(intakeRoller, 5000.0).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AuxRT",
            new RunIntakeRollerAtRpmCommand(intakeRoller, 5000.0).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AuxRTIntakeRollerForward1s",
            new RunIntakeRollerAtRpmCommand(intakeRoller, 5000.0).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AutoIntakeRollerReverse1s",
            new RunIntakeRollerAtRpmCommand(intakeRoller, -3000.0).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AuxLT",
            new RunIntakeRollerAtRpmCommand(intakeRoller, -3000.0).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AuxLTIntakeRollerReverse1s",
            new RunIntakeRollerAtRpmCommand(intakeRoller, -3000.0).withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AutoConveyorShoot1s",
            new RunConveyorPercentCommand(conveyor, Constants.RobotContainer.SHOOT_CONVEYOR_PERCENT)
                .withTimeout(1.0)
        );
        NamedCommands.registerCommand(
            "AutoSetIntakeArmTop",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AuxY",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AuxYSetIntakeArmTop",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoSetIntakeArmBottom",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AuxA",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AuxASetIntakeArmBottom",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoSetIntakeArmBottom3s",
            Commands.waitSeconds(3.0)
        );
        NamedCommands.registerCommand(
            "AutoRunClimberUp0p5s",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoRunClimberDown0p5s",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoManualIntakeArmUp0p5s",
            Commands.waitSeconds(0.5)
        );
        NamedCommands.registerCommand(
            "AutoManualIntakeArmDown0p5s",
            Commands.waitSeconds(0.5)
        );
        NamedCommands.registerCommand(
            "AutoClimberSequence",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "TimedShootFromDistanceCommand",
            buildTimedAutoShootFromDistance(2.5)
        );
        NamedCommands.registerCommand(
            "AlignToClimbPostCommand",
            Commands.none()
        );
        NamedCommands.registerCommand(
            "AutoClimberSequenceCommand",
            Commands.none()
        );
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }

    public Command buildTimedAutoShootFromDistance(double durationSec) {
        return new TimedShootAndFeedWhenReadyCommand(
            shooterFlywheel,
            conveyor,
            this::getPresetShotRPM,
            durationSec
        );
    }

    public Command buildTimedAutoShootAtRpm(double rpm, double durationSec) {
        return new TimedShootAndFeedWhenReadyCommand(
            shooterFlywheel,
            conveyor,
            () -> rpm,
            durationSec
        );
    }

    public Command buildAutoShootFeedWhenReady(double durationSec) {
        return new AutoShootFeedWhenReadyCommand(
            shooterFlywheel,
            conveyor,
            this::getPresetShotRPM,
            durationSec
        );
    }

    public Command buildSpinShooterAtPresetRpmAuto(double durationSec) {
        return new SpinShooterAtRpmCommand(
            shooterFlywheel,
            this::getPresetShotRPM
        ).withTimeout(Math.max(0.0, durationSec));
    }

    public Command buildAuxBackDistanceShootFeedCommand() {
        return new AutoGetDistanceShootFeedCommand(
            shooterFlywheel,
            conveyor,
            this::getDistanceBasedShootRPM,
            this::updateHubTagLedFeedback,
            this::clearHubTagLedFeedback
        );
    }

    public Command buildAutoDistShootFeedAuto(double durationSec) {
        return buildAuxBackDistanceShootFeedCommand()
            .withTimeout(Math.max(0.0, durationSec));
    }

    public Command buildAutoAlignFieldCentric(AutoAlignFieldCentric.AlignMode mode) {
        return new AutoAlignFieldCentric(drivetrain, mode)
            .withTimeout(Constants.RobotContainer.FIELD_CENTRIC_ALIGN_TIMEOUT_SEC);
    }

    public void updateTelemetry() {
        refreshDashboardControlState();
        telemetry.telemeterize(drivetrain.getState());
    }

    public void setLedState(BlinkinLedSubsystem.LedState state) {
        ledSubsystem.setState(state);
        SmartDashboard.putString("LED/State", state.name());
    }

    public void setVisionOdometryEnabled(boolean enabled) {
        drivetrain.setVisionOdometryEnabled(enabled);
        SmartDashboard.putBoolean("Drive/VisionOdometryEnabled", enabled);
    }

    private SendableChooser<Command> configurePathPlannerAutonomous() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("No Auto", Commands.none());

        try {
            RobotConfig robotConfig = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                drivetrain::getPose,
                pose -> {
                    if (pose == null) {
                        DriverStation.reportWarning(
                            "PathPlanner requested resetPose with null pose; skipping odometry reset.",
                            false
                        );
                        return;
                    }
                    SmartDashboard.putNumber("Auto/ResetPoseX", pose.getX());
                    SmartDashboard.putNumber("Auto/ResetPoseY", pose.getY());
                    SmartDashboard.putNumber("Auto/ResetPoseHeadingDeg", pose.getRotation().getDegrees());
                    SmartDashboard.putString("Auto/ResetPoseAlliance", AllianceUtil.getAllianceLabel());
                    SmartDashboard.putBoolean("Auto/ResetPoseFlipRed", AllianceUtil.isRed());
                    drivetrain.resetPose(pose);
                },
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.setControl(
                    autoRequest
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.1)
                ),
                robotConfig,
                AllianceUtil::isRed,
                drivetrain
            );

            List<String> autoNames = new ArrayList<>(AutoBuilder.getAllAutoNames());
            Collections.sort(autoNames);

            String preferredDefault = autoNames.contains("TEST")
                ? "TEST"
                : autoNames.contains("AutoShootThenHangLeftSide")
                    ? "AutoShootThenHangLeftSide"
                    : null;

            boolean addedAnyAuto = false;
            for (String autoName : autoNames) {
                try {
                    Command autoCommand = AutoBuilder.buildAuto(autoName);

                    if (!addedAnyAuto || autoName.equals(preferredDefault)) {
                        chooser.setDefaultOption(autoName, autoCommand);
                    } else {
                        chooser.addOption(autoName, autoCommand);
                    }
                    addedAnyAuto = true;
                } catch (RuntimeException autoEx) {
                    DriverStation.reportError(
                        "Skipping invalid PathPlanner auto '" + autoName + "': " + autoEx.getMessage(),
                        autoEx.getStackTrace()
                    );
                }
            }

            if (!addedAnyAuto) {
                DriverStation.reportError(
                    "No valid PathPlanner autos loaded. Keeping 'No Auto' fallback.",
                    false
                );
            }
        } catch (Exception ex) {
            DriverStation.reportError(
                "Failed to initialize PathPlanner autonomous: " + ex.getMessage(),
                ex.getStackTrace()
            );
        }

        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }
}
