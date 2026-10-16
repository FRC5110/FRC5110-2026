package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private static final CANBus FLYWHEEL_CAN_BUS = new CANBus(Constants.CanBus.RIO);
    private static final double HARD_MIN_RPM = Constants.ShooterTune.MIN_RPM_DEFAULT;
    private static final double HARD_MAX_RPM = Constants.ShooterTune.MAX_RPM_DEFAULT;
    private static final double TARGET_CHANGE_RESET_RPM = 10.0;
    private static final double MIN_COMMAND_RPM = 1e-3;
    private static final int FLYWHEEL_SLOT = 0;

    private final double[] distanceBreakpointsMeters =
        Arrays.copyOf(
            Constants.ShooterTune.DISTANCE_POINT_METERS_DEFAULTS,
            Constants.ShooterTune.DISTANCE_POINT_METERS_DEFAULTS.length
        );
    private final double[] rpmBreakpoints =
        Arrays.copyOf(
            Constants.ShooterTune.RPM_POINT_DEFAULTS,
            Constants.ShooterTune.RPM_POINT_DEFAULTS.length
        );
    private final InterpolatingDoubleTreeMap distanceToRpmLookup =
        new InterpolatingDoubleTreeMap();

    private final TalonFX flywheelLeaderMotor;
    private final TalonFX flywheelFollowerMotor;
    private final SparkFlex feedWheelMotor;
    private final SparkClosedLoopController feedWheelVelocityController;
    private final VelocityTorqueCurrentFOC flywheelVelocityTorqueCurrentRequest;
    private final VelocityVoltage flywheelVelocityVoltageFallbackRequest;
    private final VoltageOut flywheelCharacterizationRequest;
    private final Follower flywheelFollowerRequest;
    private final NeutralOut flywheelNeutralRequest = new NeutralOut();

    private final StatusSignal<AngularVelocity> flywheelVelocitySignal;
    private final StatusSignal<Angle> flywheelPositionSignal;
    private final StatusSignal<Voltage> flywheelMotorVoltageSignal;
    private final StatusSignal<Current> flywheelTorqueCurrentSignal;
    private final StatusSignal<Boolean> flywheelUnlicensedFeatureSignal;
    private final StatusSignal<Boolean> flywheelLeaderHardwareFaultSignal;
    private final StatusSignal<Boolean> flywheelFollowerHardwareFaultSignal;

    private final SysIdRoutine shooterSysIdRoutine;

    private double requestedTargetRPM = 0.0;
    private double activeTargetRPM = 0.0;
    private double feedWheelTargetRPM = 0.0;
    private boolean controlEnabled = false;
    private boolean usingBoost = false;
    private boolean characterizationMode = false;
    private boolean flywheelVoltageFallbackActive = false;
    private boolean flywheelFallbackWarningReported = false;
    private double rpmSlope = 0.0;
    private double lastRPM = 0.0;
    private double lastTimestampSec = 0.0;
    private double atSpeedStartSec = Double.NaN;
    private boolean atSpeedReady = false;
    private double filteredFlywheelRPM = 0.0;
    private double feedReadyStartSec = Double.NaN;
    private boolean feedReady = false;
    private DoubleSupplier distanceRpmTrimSupplier =
        () -> Constants.ShooterTune.DISTANCE_RPM_TRIM_DEFAULT;
    private double flywheelCurrentRPM = 0.0;
    private double flywheelPositionRotations = 0.0;
    private double flywheelAppliedVoltage = 0.0;
    private double flywheelTorqueCurrentAmps = 0.0;
    private boolean flywheelUnlicensedFeatureFault = false;
    private boolean flywheelLeaderHardwareFault = false;
    private boolean flywheelFollowerHardwareFault = false;
    private double feedWheelCurrentRPM = 0.0;
    private double feedWheelFaultConditionStartSec = Double.NaN;
    private boolean feedWheelHealthy = true;
    private String readyToFeedSource = "Disabled";
    private double lastTelemetryPublishSec = Double.NEGATIVE_INFINITY;
    private BooleanSupplier pitTelemetryEnabledSupplier = () -> false;
    private BooleanSupplier debugTelemetryEnabledSupplier = () -> false;
    private final boolean flywheelLeaderAvailable;
    private final boolean flywheelFollowerAvailable;
    private final boolean feedWheelAvailable;

    @SuppressWarnings("removal")
    public ShooterFlywheelSubsystem() {
        flywheelLeaderMotor = new TalonFX(
            Constants.CanIds.SHOOTER_FLYWHEEL_LEADER,
            FLYWHEEL_CAN_BUS
        );
        flywheelFollowerMotor = new TalonFX(
            Constants.CanIds.SHOOTER_FLYWHEEL_FOLLOWER,
            FLYWHEEL_CAN_BUS
        );
        feedWheelMotor = new SparkFlex(Constants.CanIds.SHOOTER_FEED_WHEEL, MotorType.kBrushless);

        flywheelLeaderAvailable =
            configureFlywheelMotor("leader", flywheelLeaderMotor, createFlywheelLeaderConfig());
        flywheelFollowerAvailable =
            configureFlywheelMotor("follower", flywheelFollowerMotor, createFlywheelFollowerConfig());
        feedWheelAvailable = configureFeedWheelMotor();

        feedWheelVelocityController = feedWheelMotor.getClosedLoopController();
        flywheelVelocityTorqueCurrentRequest =
            new VelocityTorqueCurrentFOC(0.0)
                .withSlot(FLYWHEEL_SLOT)
                .withUpdateFreqHz(Constants.ShooterTune.FLYWHEEL_CONTROL_UPDATE_HZ_DEFAULT);
        flywheelVelocityVoltageFallbackRequest =
            new VelocityVoltage(0.0)
                .withSlot(FLYWHEEL_SLOT)
                .withEnableFOC(true)
                .withUpdateFreqHz(Constants.ShooterTune.FLYWHEEL_CONTROL_UPDATE_HZ_DEFAULT);
        flywheelCharacterizationRequest =
            new VoltageOut(0.0)
                .withEnableFOC(true)
                .withUpdateFreqHz(Constants.ShooterTune.FLYWHEEL_CONTROL_UPDATE_HZ_DEFAULT);
        flywheelFollowerRequest =
            new Follower(
                Constants.CanIds.SHOOTER_FLYWHEEL_LEADER,
                Constants.ShooterTune.FLYWHEEL_FOLLOWER_OPPOSES_LEADER_DEFAULT
                    ? MotorAlignmentValue.Opposed
                    : MotorAlignmentValue.Aligned
            ).withUpdateFreqHz(Constants.ShooterTune.FLYWHEEL_CONTROL_UPDATE_HZ_DEFAULT);

        flywheelVelocitySignal = flywheelLeaderMotor.getVelocity(false).clone();
        flywheelPositionSignal = flywheelLeaderMotor.getPosition(false).clone();
        flywheelMotorVoltageSignal = flywheelLeaderMotor.getMotorVoltage(false).clone();
        flywheelTorqueCurrentSignal = flywheelLeaderMotor.getTorqueCurrent(false).clone();
        flywheelUnlicensedFeatureSignal = flywheelLeaderMotor.getFault_UnlicensedFeatureInUse(false).clone();
        flywheelLeaderHardwareFaultSignal = flywheelLeaderMotor.getFault_Hardware(false).clone();
        flywheelFollowerHardwareFaultSignal = flywheelFollowerMotor.getFault_Hardware(false).clone();

        configureSignalUpdateRates();
        if (flywheelLeaderAvailable) {
            flywheelLeaderMotor.optimizeBusUtilization(Constants.ShooterTune.FLYWHEEL_STATUS_UPDATE_HZ_DEFAULT);
        }
        if (flywheelFollowerAvailable) {
            flywheelFollowerMotor.optimizeBusUtilization(Constants.ShooterTune.FLYWHEEL_STATUS_UPDATE_HZ_DEFAULT);
        }
        if (feedWheelAvailable) {
            reportFeedWheelMotorFirmware("feed wheel", feedWheelMotor);
        }
        reportHardwareAvailability();
        refreshDistanceTableFromConstants();
        refreshFlywheelSignals();
        feedWheelCurrentRPM = feedWheelAvailable ? readFeedWheelWheelRpm() : 0.0;
        lastTimestampSec = Timer.getFPGATimestamp();
        lastRPM = flywheelCurrentRPM;
        filteredFlywheelRPM = flywheelCurrentRPM;

        shooterSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Constants.ShooterTune.SYSID_RAMP_RATE_VOLTS_PER_SEC_DEFAULT).per(Second),
                Volts.of(Constants.ShooterTune.SYSID_STEP_VOLTAGE_DEFAULT),
                Seconds.of(Constants.ShooterTune.SYSID_TIMEOUT_SEC_DEFAULT)
            ),
            new SysIdRoutine.Mechanism(
                volts -> applyCharacterizationVoltage(volts.in(Volts)),
                log -> log.motor("shooter-flywheel")
                    .voltage(Volts.of(getAppliedVoltage()))
                    .angularPosition(Rotations.of(getPositionRotations()))
                    .angularVelocity(RotationsPerSecond.of(getCurrentRPM() / 60.0)),
                this,
                "shooter-flywheel"
            )
        );
    }

    public void setTargetRPM(double rpm) {
        double clampedRPM = clampToConfiguredRange(rpm);

        if (Math.abs(clampedRPM) < MIN_COMMAND_RPM) {
            stop();
            return;
        }

        if (!controlEnabled || Math.abs(clampedRPM - requestedTargetRPM) > TARGET_CHANGE_RESET_RPM) {
            resetAtSpeedState();
            resetFeedReadyState(getCurrentRPM());
        }

        characterizationMode = false;
        requestedTargetRPM = clampedRPM;
        controlEnabled = true;
    }

    public void setTargetFromDistanceMeters(double distanceMeters) {
        setTargetRPM(getRecommendedRPMForDistanceMeters(distanceMeters));
    }

    public double getRecommendedRPMForDistanceMeters(double distanceMeters) {
        double baseRecommendedRpm = getBaseRecommendedRPMForDistanceMeters(distanceMeters);
        if (!Double.isFinite(baseRecommendedRpm) || Math.abs(baseRecommendedRpm) < MIN_COMMAND_RPM) {
            return 0.0;
        }

        return clampToConfiguredRange(baseRecommendedRpm + getDistanceRpmTrimRPM());
    }

    public double getBaseRecommendedRPMForDistanceMeters(double distanceMeters) {
        Double recommendedRpm = distanceToRpmLookup.get(distanceMeters);
        if (recommendedRpm == null || !Double.isFinite(recommendedRpm)) {
            return 0.0;
        }
        return recommendedRpm;
    }

    public void setDistanceRpmTrimSupplier(DoubleSupplier trimSupplier) {
        if (trimSupplier == null) {
            distanceRpmTrimSupplier = () -> Constants.ShooterTune.DISTANCE_RPM_TRIM_DEFAULT;
            return;
        }

        distanceRpmTrimSupplier = trimSupplier;
    }

    public double getDistanceRpmTrimRPM() {
        double trimRpm = distanceRpmTrimSupplier.getAsDouble();
        return Double.isFinite(trimRpm) ? trimRpm : Constants.ShooterTune.DISTANCE_RPM_TRIM_DEFAULT;
    }

    public void setTelemetryEnabledSuppliers(
        BooleanSupplier pitTelemetryEnabledSupplier,
        BooleanSupplier debugTelemetryEnabledSupplier
    ) {
        this.pitTelemetryEnabledSupplier =
            pitTelemetryEnabledSupplier == null ? () -> false : pitTelemetryEnabledSupplier;
        this.debugTelemetryEnabledSupplier =
            debugTelemetryEnabledSupplier == null ? () -> false : debugTelemetryEnabledSupplier;
    }

    public double getCurrentRPM() {
        return getFlywheelCurrentRPM();
    }

    public double getFlywheelCurrentRPM() {
        return flywheelCurrentRPM;
    }

    public double getFeedWheelCurrentRPM() {
        return feedWheelCurrentRPM;
    }

    public double getPositionRotations() {
        return flywheelPositionRotations;
    }

    public double getPositionDegrees() {
        return getPositionRotations() * 360.0;
    }

    public double getTargetRPM() {
        return getFlywheelTargetRPM();
    }

    public double getFlywheelTargetRPM() {
        return activeTargetRPM;
    }

    public double getFeedWheelTargetRPM() {
        return feedWheelTargetRPM;
    }

    public boolean atSpeed() {
        return atSpeedReady;
    }

    public boolean atSpeed(double toleranceRPM) {
        if (!flywheelLeaderAvailable || !controlEnabled) {
            return false;
        }

        return Math.abs(getFlywheelCurrentRPM())
            >= (Math.abs(activeTargetRPM) - Math.abs(toleranceRPM));
    }

    public boolean isUsingBoost() {
        return usingBoost;
    }

    public boolean isReadyToFeed() {
        return computeReadyToFeed(
            feedReady,
            isFeedWheelCommanded(feedWheelTargetRPM),
            feedWheelHealthy
        );
    }

    public boolean isBelowFeedDropoutThreshold() {
        if (!controlEnabled) {
            return true;
        }

        double targetRPM = Math.abs(activeTargetRPM);
        if (targetRPM < MIN_COMMAND_RPM) {
            return true;
        }

        double currentRPM = Math.abs(filteredFlywheelRPM);
        return currentRPM < (targetRPM - Math.abs(Constants.ShooterTune.FEED_DROPOUT_TOL_RPM_DEFAULT));
    }

    public boolean isFeedWheelHealthy() {
        return feedWheelHealthy;
    }

    public void stop() {
        characterizationMode = false;
        controlEnabled = false;
        usingBoost = false;
        requestedTargetRPM = 0.0;
        activeTargetRPM = 0.0;
        feedWheelTargetRPM = 0.0;
        resetAtSpeedState();
        resetFeedReadyState(getCurrentRPM());
        readyToFeedSource = "Disabled";
        feedWheelFaultConditionStartSec = Double.NaN;
        feedWheelHealthy = true;
        flywheelVoltageFallbackActive = false;
        stopFeedWheelOutput();
        stopFlywheelOutput();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine
            .quasistatic(direction)
            .beforeStarting(this::beginCharacterization)
            .finallyDo(this::endCharacterization);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine
            .dynamic(direction)
            .beforeStarting(this::beginCharacterization)
            .finallyDo(this::endCharacterization);
    }

    @Override
    public void periodic() {
        refreshFlywheelSignals();
        feedWheelCurrentRPM = feedWheelAvailable ? readFeedWheelWheelRpm() : 0.0;

        double nowSec = Timer.getFPGATimestamp();
        double currentRPM = flywheelCurrentRPM;

        updateSlope(nowSec, currentRPM);
        updateFlywheelReadyFilter(currentRPM);

        if (DriverStation.isDisabled() && !controlEnabled && !characterizationMode) {
            activeTargetRPM = 0.0;
            feedWheelTargetRPM = 0.0;
            usingBoost = false;
            lastTimestampSec = nowSec;
            lastRPM = currentRPM;
            filteredFlywheelRPM = currentRPM;
            resetAtSpeedState();
            resetFeedReadyState(currentRPM);
            updateFeedWheelHealth(nowSec, 0.0, feedWheelCurrentRPM);
            readyToFeedSource = "Disabled";
            stopFeedWheelOutput();
            stopFlywheelOutput();
            publishTelemetry(nowSec);
            return;
        }

        if (characterizationMode) {
            activeTargetRPM = 0.0;
            feedWheelTargetRPM = 0.0;
            usingBoost = false;
            resetAtSpeedState();
            resetFeedReadyState(currentRPM);
            updateFeedWheelHealth(nowSec, 0.0, feedWheelCurrentRPM);
            readyToFeedSource = "Characterization";
            publishTelemetry(nowSec);
            return;
        }

        if (!controlEnabled) {
            activeTargetRPM = 0.0;
            feedWheelTargetRPM = 0.0;
            usingBoost = false;
            resetAtSpeedState();
            resetFeedReadyState(currentRPM);
            updateFeedWheelHealth(nowSec, 0.0, feedWheelCurrentRPM);
            readyToFeedSource = "Disabled";
            stopFeedWheelOutput();
            stopFlywheelOutput();
        } else {
            activeTargetRPM = getActiveTargetRPM();
            feedWheelTargetRPM = computeClampedFeedWheelTargetRPM(activeTargetRPM);
            runControlLoop(activeTargetRPM, feedWheelTargetRPM);
            updateAtSpeedState(nowSec, currentRPM);
            updateFeedReadyState(nowSec);
            updateFeedWheelHealth(nowSec, feedWheelTargetRPM, feedWheelCurrentRPM);
            readyToFeedSource = determineReadyToFeedSource();
        }

        publishTelemetry(nowSec);
    }

    static double computeFeedWheelTargetRPM(
        double flywheelTargetRPM,
        double surfaceSpeedRatio,
        double multiplier,
        double offsetRPM
    ) {
        if (!Double.isFinite(flywheelTargetRPM) || Math.abs(flywheelTargetRPM) < MIN_COMMAND_RPM) {
            return 0.0;
        }

        double targetRPM = flywheelTargetRPM * surfaceSpeedRatio * multiplier;
        if (Double.isFinite(offsetRPM)) {
            targetRPM += offsetRPM;
        }
        return targetRPM;
    }

    static double computeFixedFeedWheelTargetRPM(double flywheelTargetRPM, double commandedFeedWheelRPM) {
        if (!Double.isFinite(flywheelTargetRPM) || Math.abs(flywheelTargetRPM) < MIN_COMMAND_RPM) {
            return 0.0;
        }
        if (!Double.isFinite(commandedFeedWheelRPM) || Math.abs(commandedFeedWheelRPM) < MIN_COMMAND_RPM) {
            return 0.0;
        }
        return Math.copySign(Math.abs(commandedFeedWheelRPM), flywheelTargetRPM);
    }

    static boolean hasFeedWheelHardFaultCondition(
        double targetRPM,
        double measuredRPM,
        double minCommandRPM,
        double minMeasuredRPM
    ) {
        return Math.abs(targetRPM) >= Math.abs(minCommandRPM)
            && (!Double.isFinite(measuredRPM) || Math.abs(measuredRPM) < Math.abs(minMeasuredRPM));
    }

    static boolean computeReadyToFeed(
        boolean flywheelReadyToFeed,
        boolean feedWheelCommanded,
        boolean feedWheelHealthy
    ) {
        return flywheelReadyToFeed && (!feedWheelCommanded || feedWheelHealthy);
    }

    private TalonFXConfiguration createFlywheelLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.ShooterTune.FLYWHEEL_LEADER_INVERTED_DEFAULT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = Constants.ShooterTune.FLYWHEEL_BRAKE_MODE_DEFAULT
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit =
            Constants.ShooterTune.FLYWHEEL_STATOR_CURRENT_LIMIT_AMPS_DEFAULT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
            Constants.ShooterTune.FLYWHEEL_SUPPLY_CURRENT_LIMIT_AMPS_DEFAULT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit =
            Constants.ShooterTune.FLYWHEEL_SUPPLY_CURRENT_LOWER_LIMIT_AMPS_DEFAULT;
        config.CurrentLimits.SupplyCurrentLowerTime =
            Constants.ShooterTune.FLYWHEEL_SUPPLY_CURRENT_LOWER_TIME_SEC_DEFAULT;
        config.TorqueCurrent.PeakForwardTorqueCurrent =
            Constants.ShooterTune.FLYWHEEL_PEAK_FORWARD_TORQUE_CURRENT_AMPS_DEFAULT;
        config.TorqueCurrent.PeakReverseTorqueCurrent =
            Constants.ShooterTune.FLYWHEEL_PEAK_REVERSE_TORQUE_CURRENT_AMPS_DEFAULT;
        config.Feedback.SensorToMechanismRatio =
            sanitizePositiveRatio(Constants.ShooterTune.FLYWHEEL_MOTOR_TO_WHEEL_RATIO_DEFAULT);
        config.Slot0.kP = Constants.ShooterTune.FLYWHEEL_SLOT0_KP_DEFAULT;
        config.Slot0.kI = Constants.ShooterTune.FLYWHEEL_SLOT0_KI_DEFAULT;
        config.Slot0.kD = Constants.ShooterTune.FLYWHEEL_SLOT0_KD_DEFAULT;
        config.Slot0.kS = Constants.ShooterTune.FLYWHEEL_SLOT0_KS_DEFAULT;
        config.Slot0.kV = Constants.ShooterTune.FLYWHEEL_SLOT0_KV_DEFAULT;
        config.Slot0.kA = Constants.ShooterTune.FLYWHEEL_SLOT0_KA_DEFAULT;
        return config;
    }

    private TalonFXConfiguration createFlywheelFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.ShooterTune.FLYWHEEL_BRAKE_MODE_DEFAULT
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit =
            Constants.ShooterTune.FLYWHEEL_STATOR_CURRENT_LIMIT_AMPS_DEFAULT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
            Constants.ShooterTune.FLYWHEEL_SUPPLY_CURRENT_LIMIT_AMPS_DEFAULT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit =
            Constants.ShooterTune.FLYWHEEL_SUPPLY_CURRENT_LOWER_LIMIT_AMPS_DEFAULT;
        config.CurrentLimits.SupplyCurrentLowerTime =
            Constants.ShooterTune.FLYWHEEL_SUPPLY_CURRENT_LOWER_TIME_SEC_DEFAULT;
        return config;
    }

    @SuppressWarnings("removal")
    private boolean configureFeedWheelMotor() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(
            Constants.ShooterTune.FEED_WHEEL_BRAKE_MODE_DEFAULT
                ? IdleMode.kBrake
                : IdleMode.kCoast
        );
        config.smartCurrentLimit(Constants.ShooterTune.FEED_WHEEL_CURRENT_LIMIT_AMPS_DEFAULT);
        config.inverted(Constants.ShooterTune.FEED_WHEEL_INVERTED_DEFAULT);
        config.closedLoop.p(Constants.ShooterTune.FEED_WHEEL_PID_P_DEFAULT);
        config.closedLoop.i(Constants.ShooterTune.FEED_WHEEL_PID_I_DEFAULT);
        config.closedLoop.d(Constants.ShooterTune.FEED_WHEEL_PID_D_DEFAULT);
        config.closedLoop.velocityFF(Constants.ShooterTune.FEED_WHEEL_PID_FF_DEFAULT);

        try {
            feedWheelMotor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            );
            return true;
        } catch (RuntimeException ex) {
            DriverStation.reportError(
                "Failed to configure shooter feed wheel Spark Flex CAN ID "
                    + feedWheelMotor.getDeviceId()
                    + " (firmware "
                    + feedWheelMotor.getFirmwareString()
                    + "): "
                    + ex.getMessage(),
                ex.getStackTrace()
            );
            return false;
        }
    }

    private boolean configureFlywheelMotor(String role, TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = motor.getConfigurator().apply(config);
        if (!status.isOK()) {
            DriverStation.reportError(
                "Failed to configure shooter "
                    + role
                    + " TalonFX CAN ID "
                    + motor.getDeviceID()
                    + ": "
                    + status,
                false
            );
            return false;
        }
        return true;
    }

    private double getActiveTargetRPM() {
        if (!Constants.ShooterTune.OVERRIDE_ENABLED_DEFAULT) {
            return requestedTargetRPM;
        }

        return clampToConfiguredRange(Constants.ShooterTune.OVERRIDE_RPM_DEFAULT);
    }

    private void runControlLoop(double flywheelTargetRPM, double feedWheelTargetRPM) {
        if (!flywheelLeaderAvailable || Math.abs(flywheelTargetRPM) < MIN_COMMAND_RPM) {
            stopFeedWheelOutput();
            stopFlywheelOutput();
            return;
        }

        runFlywheelVelocityControl(flywheelTargetRPM);
        runFeedWheelVelocityControl(feedWheelTargetRPM);
        usingBoost = false;
    }

    private void runFlywheelVelocityControl(double flywheelTargetRPM) {
        if (!flywheelLeaderAvailable) {
            stopFlywheelOutput();
            return;
        }

        applyFlywheelFollowerRequest();

        double flywheelTargetRps = rpmToRps(flywheelTargetRPM);
        if (flywheelUnlicensedFeatureFault) {
            flywheelVoltageFallbackActive = true;
            warnIfFlywheelFallbackActive();
            flywheelLeaderMotor.setControl(
                flywheelVelocityVoltageFallbackRequest.withVelocity(flywheelTargetRps)
            );
            return;
        }

        StatusCode controlStatus = flywheelLeaderMotor.setControl(
            flywheelVelocityTorqueCurrentRequest.withVelocity(flywheelTargetRps)
        );
        if (controlStatus.isOK()) {
            flywheelVoltageFallbackActive = false;
            return;
        }

        flywheelVoltageFallbackActive = true;
        warnIfFlywheelFallbackActive();
        flywheelLeaderMotor.setControl(
            flywheelVelocityVoltageFallbackRequest.withVelocity(flywheelTargetRps)
        );
    }

    private void runFeedWheelVelocityControl(double feedWheelTargetRPM) {
        if (!feedWheelAvailable) {
            stopFeedWheelOutput();
            return;
        }

        double clampedTargetRPM = clampFeedWheelTargetRPM(feedWheelTargetRPM);
        if (Math.abs(clampedTargetRPM) < MIN_COMMAND_RPM) {
            stopFeedWheelOutput();
            return;
        }

        feedWheelVelocityController.setReference(
            wheelRpmToMotorRpm(
                clampedTargetRPM,
                Constants.ShooterTune.FEED_WHEEL_MOTOR_TO_WHEEL_RATIO_DEFAULT
            ),
            SparkFlex.ControlType.kVelocity
        );
    }

    private void updateSlope(double nowSec, double currentRPM) {
        double dtSec = nowSec - lastTimestampSec;
        if (dtSec > 1e-4) {
            rpmSlope = (currentRPM - lastRPM) / dtSec;
        } else {
            rpmSlope = 0.0;
        }
        lastTimestampSec = nowSec;
        lastRPM = currentRPM;
    }

    private void updateFlywheelReadyFilter(double currentRPM) {
        double alpha = MathUtil.clamp(Constants.ShooterTune.FEED_RPM_FILTER_ALPHA_DEFAULT, 0.0, 1.0);

        if (!Double.isFinite(filteredFlywheelRPM) || alpha <= 1e-6) {
            filteredFlywheelRPM = currentRPM;
            return;
        }

        filteredFlywheelRPM += alpha * (currentRPM - filteredFlywheelRPM);
    }

    private void updateAtSpeedState(double nowSec, double currentRPM) {
        if (!controlEnabled || Math.abs(activeTargetRPM) < MIN_COMMAND_RPM) {
            resetAtSpeedState();
            return;
        }

        double toleranceRPM = Math.abs(Constants.ShooterTune.AT_SPEED_TOL_RPM_DEFAULT);
        double stableSec = Math.max(0.0, Constants.ShooterTune.AT_SPEED_STABLE_SEC_DEFAULT);
        double readyThresholdRPM = Math.abs(activeTargetRPM) - toleranceRPM;

        if (Math.abs(currentRPM) >= readyThresholdRPM) {
            if (Double.isNaN(atSpeedStartSec)) {
                atSpeedStartSec = nowSec;
            }
            atSpeedReady = (nowSec - atSpeedStartSec) >= stableSec;
        } else {
            resetAtSpeedState();
        }
    }

    private void resetAtSpeedState() {
        atSpeedStartSec = Double.NaN;
        atSpeedReady = false;
    }

    private void updateFeedReadyState(double nowSec) {
        if (!controlEnabled || Math.abs(activeTargetRPM) < MIN_COMMAND_RPM || shouldPauseFeedWhileBoosting()) {
            resetFeedReadyState(filteredFlywheelRPM);
            return;
        }

        double readyThresholdRPM =
            Math.abs(activeTargetRPM) - Math.abs(Constants.ShooterTune.FEED_READY_TOL_RPM_DEFAULT);
        double stableSec = Math.max(0.0, Constants.ShooterTune.FEED_READY_STABLE_SEC_DEFAULT);

        if (Math.abs(filteredFlywheelRPM) >= readyThresholdRPM) {
            if (Double.isNaN(feedReadyStartSec)) {
                feedReadyStartSec = nowSec;
            }
            feedReady = (nowSec - feedReadyStartSec) >= stableSec;
        } else {
            resetFeedReadyState(filteredFlywheelRPM);
        }
    }

    private void resetFeedReadyState(double currentRPM) {
        filteredFlywheelRPM = currentRPM;
        feedReadyStartSec = Double.NaN;
        feedReady = false;
    }

    private void updateFeedWheelHealth(double nowSec, double targetRPM, double currentRPM) {
        if (!feedWheelAvailable) {
            feedWheelFaultConditionStartSec = Double.NaN;
            feedWheelHealthy = !isFeedWheelCommanded(targetRPM);
            return;
        }

        boolean hardFaultCondition = hasFeedWheelHardFaultCondition(
            targetRPM,
            currentRPM,
            Constants.ShooterTune.FEED_WHEEL_HEALTH_MIN_COMMAND_RPM_DEFAULT,
            Constants.ShooterTune.FEED_WHEEL_HEALTH_MIN_MEASURED_RPM_DEFAULT
        );

        if (!hardFaultCondition) {
            feedWheelFaultConditionStartSec = Double.NaN;
            feedWheelHealthy = true;
            return;
        }

        if (Double.isNaN(feedWheelFaultConditionStartSec)) {
            feedWheelFaultConditionStartSec = nowSec;
        }

        feedWheelHealthy =
            (nowSec - feedWheelFaultConditionStartSec)
                < Math.max(0.0, Constants.ShooterTune.FEED_WHEEL_HEALTH_FAULT_DELAY_SEC_DEFAULT);
    }

    private void refreshDistanceTableFromConstants() {
        distanceToRpmLookup.clear();
        for (int i = 0; i < distanceBreakpointsMeters.length; i++) {
            distanceBreakpointsMeters[i] = Constants.ShooterTune.DISTANCE_POINT_METERS_DEFAULTS[i];
            rpmBreakpoints[i] = Constants.ShooterTune.RPM_POINT_DEFAULTS[i];
            if (Double.isFinite(distanceBreakpointsMeters[i]) && Double.isFinite(rpmBreakpoints[i])) {
                distanceToRpmLookup.put(distanceBreakpointsMeters[i], rpmBreakpoints[i]);
            }
        }
    }

    private double clampToConfiguredRange(double rpm) {
        double minRpm = MathUtil.clamp(Constants.ShooterTune.MIN_RPM_DEFAULT, HARD_MIN_RPM, HARD_MAX_RPM);
        double maxRpm = MathUtil.clamp(Constants.ShooterTune.MAX_RPM_DEFAULT, HARD_MIN_RPM, HARD_MAX_RPM);

        if (minRpm > maxRpm) {
            double swap = minRpm;
            minRpm = maxRpm;
            maxRpm = swap;
        }

        return MathUtil.clamp(rpm, minRpm, maxRpm);
    }

    private double computeClampedFeedWheelTargetRPM(double flywheelTargetRPM) {
        return clampFeedWheelTargetRPM(
            computeFixedFeedWheelTargetRPM(
                flywheelTargetRPM,
                Constants.ShooterTune.FEED_WHEEL_WHEN_SHOOTING_RPM_DEFAULT
            )
        );
    }

    private double clampFeedWheelTargetRPM(double targetRPM) {
        return MathUtil.clamp(
            targetRPM,
            -Math.abs(Constants.ShooterTune.FEED_WHEEL_MAX_RPM_DEFAULT),
            Math.abs(Constants.ShooterTune.FEED_WHEEL_MAX_RPM_DEFAULT)
        );
    }

    private void beginCharacterization() {
        stop();
        characterizationMode = true;
    }

    private void endCharacterization() {
        stop();
    }

    private void applyCharacterizationVoltage(double volts) {
        if (!flywheelLeaderAvailable) {
            characterizationMode = false;
            return;
        }

        characterizationMode = true;
        controlEnabled = false;
        usingBoost = false;
        requestedTargetRPM = 0.0;
        activeTargetRPM = 0.0;
        feedWheelTargetRPM = 0.0;
        resetAtSpeedState();
        resetFeedReadyState(getCurrentRPM());
        stopFeedWheelOutput();
        applyFlywheelFollowerRequest();
        flywheelLeaderMotor.setControl(flywheelCharacterizationRequest.withOutput(volts));
    }

    private double getAppliedVoltage() {
        return flywheelAppliedVoltage;
    }

    private void applyFlywheelFollowerRequest() {
        if (flywheelFollowerAvailable) {
            flywheelFollowerMotor.setControl(flywheelFollowerRequest);
        }
    }

    private void stopFlywheelOutput() {
        if (flywheelLeaderAvailable) {
            flywheelLeaderMotor.setControl(flywheelNeutralRequest);
        }
        if (flywheelFollowerAvailable) {
            flywheelFollowerMotor.setControl(flywheelNeutralRequest);
        }
    }

    private void stopFeedWheelOutput() {
        if (feedWheelAvailable) {
            feedWheelMotor.set(0.0);
        }
    }

    private void refreshFlywheelSignals() {
        if (flywheelLeaderAvailable && flywheelFollowerAvailable) {
            BaseStatusSignal.refreshAll(
                flywheelVelocitySignal,
                flywheelPositionSignal,
                flywheelMotorVoltageSignal,
                flywheelTorqueCurrentSignal,
                flywheelUnlicensedFeatureSignal,
                flywheelLeaderHardwareFaultSignal,
                flywheelFollowerHardwareFaultSignal
            );
        } else if (flywheelLeaderAvailable) {
            BaseStatusSignal.refreshAll(
                flywheelVelocitySignal,
                flywheelPositionSignal,
                flywheelMotorVoltageSignal,
                flywheelTorqueCurrentSignal,
                flywheelUnlicensedFeatureSignal,
                flywheelLeaderHardwareFaultSignal
            );
        } else if (flywheelFollowerAvailable) {
            BaseStatusSignal.refreshAll(flywheelFollowerHardwareFaultSignal);
        }

        if (flywheelLeaderAvailable) {
            flywheelCurrentRPM = rpsToRpm(flywheelVelocitySignal.getValueAsDouble());
            flywheelPositionRotations = flywheelPositionSignal.getValueAsDouble();
            flywheelAppliedVoltage = flywheelMotorVoltageSignal.getValueAsDouble();
            flywheelTorqueCurrentAmps = flywheelTorqueCurrentSignal.getValueAsDouble();
            flywheelUnlicensedFeatureFault = Boolean.TRUE.equals(flywheelUnlicensedFeatureSignal.getValue());
            flywheelLeaderHardwareFault = Boolean.TRUE.equals(flywheelLeaderHardwareFaultSignal.getValue());
        } else {
            flywheelCurrentRPM = 0.0;
            flywheelPositionRotations = 0.0;
            flywheelAppliedVoltage = 0.0;
            flywheelTorqueCurrentAmps = 0.0;
            flywheelUnlicensedFeatureFault = false;
            flywheelLeaderHardwareFault = false;
        }

        flywheelFollowerHardwareFault =
            flywheelFollowerAvailable && Boolean.TRUE.equals(flywheelFollowerHardwareFaultSignal.getValue());
    }

    private double readFeedWheelWheelRpm() {
        return motorRpmToWheelRpm(
            feedWheelMotor.getEncoder().getVelocity(),
            Constants.ShooterTune.FEED_WHEEL_MOTOR_TO_WHEEL_RATIO_DEFAULT
        );
    }

    private void publishTelemetry(double nowSec) {
        double telemetryIntervalSec = Math.max(0.0, Constants.ShooterTune.TELEMETRY_PUBLISH_SEC_DEFAULT);
        if (telemetryIntervalSec > 0.0
                && Double.isFinite(lastTelemetryPublishSec)
                && (nowSec - lastTelemetryPublishSec) < telemetryIntervalSec) {
            return;
        }
        lastTelemetryPublishSec = nowSec;
        boolean tuningTelemetryEnabled =
            pitTelemetryEnabledSupplier.getAsBoolean() || debugTelemetryEnabledSupplier.getAsBoolean();
        boolean debugTelemetryEnabled = debugTelemetryEnabledSupplier.getAsBoolean();

        SmartDashboard.putNumber("Shooter/TargetRPM", activeTargetRPM);
        SmartDashboard.putNumber("Shooter/CurrentRPM", flywheelCurrentRPM);
        SmartDashboard.putNumber("Shooter/ErrorRPM", activeTargetRPM - flywheelCurrentRPM);
        SmartDashboard.putNumber("Shooter/FlywheelTargetRPM", activeTargetRPM);
        SmartDashboard.putNumber("Shooter/FlywheelCurrentRPM", flywheelCurrentRPM);
        SmartDashboard.putNumber("Shooter/FlywheelErrorRPM", activeTargetRPM - flywheelCurrentRPM);
        SmartDashboard.putBoolean("Shooter/FlywheelAtSpeed", atSpeedReady);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeedReady);
        SmartDashboard.putNumber("Shooter/FeedWheelTargetRPM", feedWheelTargetRPM);
        SmartDashboard.putNumber("Shooter/FeedWheelCurrentRPM", feedWheelCurrentRPM);
        SmartDashboard.putNumber("Shooter/FeedWheelErrorRPM", feedWheelTargetRPM - feedWheelCurrentRPM);
        SmartDashboard.putBoolean("Shooter/FeedWheelHealthy", feedWheelHealthy);
        SmartDashboard.putBoolean(
            "Shooter/FeedWheelFault",
            isFeedWheelCommanded(feedWheelTargetRPM) && !feedWheelHealthy
        );
        SmartDashboard.putBoolean("Shooter/ReadyToFeed", isReadyToFeed());
        SmartDashboard.putString("Shooter/ReadyToFeedSource", readyToFeedSource);
        SmartDashboard.putBoolean("Shooter/UsingBoost", usingBoost);
        SmartDashboard.putBoolean("Shooter/FlywheelLeaderAvailable", flywheelLeaderAvailable);
        SmartDashboard.putBoolean("Shooter/FlywheelFollowerAvailable", flywheelFollowerAvailable);
        SmartDashboard.putBoolean("Shooter/FeedWheelAvailable", feedWheelAvailable);

        if (tuningTelemetryEnabled) {
            SmartDashboard.putNumber(Constants.ShooterTune.DISTANCE_RPM_TRIM_KEY, getDistanceRpmTrimRPM());
        }

        if (debugTelemetryEnabled) {
            SmartDashboard.putNumber("Shooter/FlywheelPositionRot", flywheelPositionRotations);
            SmartDashboard.putNumber("Shooter/FlywheelAppliedVoltage", flywheelAppliedVoltage);
            SmartDashboard.putNumber("Shooter/FlywheelTorqueCurrentAmps", flywheelTorqueCurrentAmps);
            SmartDashboard.putString(
                "Shooter/FlywheelControlMode",
                flywheelVoltageFallbackActive ? "VelocityVoltageFallback" : "VelocityTorqueCurrentFOC"
            );
            SmartDashboard.putBoolean("Shooter/FlywheelLeaderHardwareFault", flywheelLeaderHardwareFault);
            SmartDashboard.putBoolean("Shooter/FlywheelFollowerHardwareFault", flywheelFollowerHardwareFault);
            SmartDashboard.putBoolean("Shooter/FlywheelUnlicensedFeatureFault", flywheelUnlicensedFeatureFault);
        }
    }

    private String determineReadyToFeedSource() {
        if (!controlEnabled) {
            return "Disabled";
        }
        if (!flywheelLeaderAvailable) {
            return "FlywheelUnavailable";
        }
        if (isFeedWheelCommanded(feedWheelTargetRPM) && !feedWheelAvailable) {
            return "FeedWheelUnavailable";
        }
        if (isFeedWheelCommanded(feedWheelTargetRPM) && !feedWheelHealthy) {
            return "FeedWheelFault";
        }
        if (feedReady) {
            return "FlywheelReady";
        }
        if (atSpeedReady) {
            return "FlywheelAtSpeed";
        }
        return "FlywheelSpinUp";
    }

    private void warnIfFlywheelFallbackActive() {
        if (flywheelFallbackWarningReported) {
            return;
        }

        flywheelFallbackWarningReported = true;
        DriverStation.reportWarning(
            "Shooter flywheel fell back to VelocityVoltage control; tune fallback gains if Phoenix Pro FOC is unavailable.",
            false
        );
    }

    public boolean shouldPauseFeedWhileBoosting() {
        return Constants.ShooterTune.PAUSE_FEED_WHEN_BOOST_DEFAULT && isUsingBoost();
    }

    private static boolean isFeedWheelCommanded(double targetRPM) {
        return Math.abs(targetRPM) >= Math.abs(Constants.ShooterTune.FEED_WHEEL_HEALTH_MIN_COMMAND_RPM_DEFAULT);
    }

    private static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    private static double rpsToRpm(double rps) {
        return rps * 60.0;
    }

    private static double sanitizePositiveRatio(double ratio) {
        if (!Double.isFinite(ratio) || Math.abs(ratio) < MIN_COMMAND_RPM) {
            return 1.0;
        }
        return Math.abs(ratio);
    }

    private static double motorRpmToWheelRpm(double motorRpm, double motorToWheelRatio) {
        return motorRpm / sanitizePositiveRatio(motorToWheelRatio);
    }

    private static double wheelRpmToMotorRpm(double wheelRpm, double motorToWheelRatio) {
        return wheelRpm * sanitizePositiveRatio(motorToWheelRatio);
    }

    private void reportFeedWheelMotorFirmware(String role, SparkFlex motor) {
        DriverStation.reportWarning(
            "Shooter "
                + role
                + " Spark Flex CAN ID "
                + motor.getDeviceId()
                + " firmware "
                + motor.getFirmwareString(),
            false
        );
    }

    private void configureSignalUpdateRates() {
        if (flywheelLeaderAvailable && flywheelFollowerAvailable) {
            BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.ShooterTune.FLYWHEEL_STATUS_UPDATE_HZ_DEFAULT,
                flywheelVelocitySignal,
                flywheelPositionSignal,
                flywheelMotorVoltageSignal,
                flywheelTorqueCurrentSignal,
                flywheelUnlicensedFeatureSignal,
                flywheelLeaderHardwareFaultSignal,
                flywheelFollowerHardwareFaultSignal
            );
            return;
        }

        if (flywheelLeaderAvailable) {
            BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.ShooterTune.FLYWHEEL_STATUS_UPDATE_HZ_DEFAULT,
                flywheelVelocitySignal,
                flywheelPositionSignal,
                flywheelMotorVoltageSignal,
                flywheelTorqueCurrentSignal,
                flywheelUnlicensedFeatureSignal,
                flywheelLeaderHardwareFaultSignal
            );
        }

        if (flywheelFollowerAvailable) {
            BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.ShooterTune.FLYWHEEL_STATUS_UPDATE_HZ_DEFAULT,
                flywheelFollowerHardwareFaultSignal
            );
        }
    }

    private void reportHardwareAvailability() {
        if (!flywheelLeaderAvailable) {
            DriverStation.reportWarning(
                "Shooter leader TalonFX unavailable at startup; shooter flywheel output disabled.",
                false
            );
        }
        if (!flywheelFollowerAvailable) {
            DriverStation.reportWarning(
                "Shooter follower TalonFX unavailable at startup; follower output disabled.",
                false
            );
        }
        if (!feedWheelAvailable) {
            DriverStation.reportWarning(
                "Shooter feed wheel Spark Flex unavailable at startup; feed wheel output disabled.",
                false
            );
        }
    }
}
