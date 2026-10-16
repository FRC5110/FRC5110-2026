package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorArmSubsystem extends SubsystemBase {

    private static final String DASHBOARD_PREFIX = "Aux/ConveyorArm/";

    private enum ArmControlMode {
        NEUTRAL,
        MANUAL,
        POSITION_HOLD
    }

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final DutyCycleOut dutyCycleRequest =
        new DutyCycleOut(0.0).withUpdateFreqHz(Constants.ConveyorArm.CONTROL_UPDATE_HZ);
    private final VoltageOut voltageRequest =
        new VoltageOut(0.0).withUpdateFreqHz(Constants.ConveyorArm.CONTROL_UPDATE_HZ);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final Follower followerRequest =
        new Follower(
            Constants.CanIds.CONVEYOR_ARM_LEADER,
            Constants.ConveyorArm.FOLLOWER_OPPOSES_LEADER
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned
        ).withUpdateFreqHz(Constants.ConveyorArm.CONTROL_UPDATE_HZ);

    private final boolean leaderAvailable;
    private final boolean followerAvailable;

    private BooleanSupplier calibrationTelemetryEnabledSupplier = () -> false;
    private BooleanSupplier debugTelemetryEnabledSupplier = () -> false;
    private final PIDController positionController =
        new PIDController(
            Constants.ConveyorArm.POSITION_KP_VOLTS_PER_DEGREE,
            Constants.ConveyorArm.POSITION_KI_VOLTS_PER_DEGREE_SECOND,
            Constants.ConveyorArm.POSITION_KD_VOLTS_PER_DEGREE_PER_SECOND
        );
    private ArmControlMode controlMode = ArmControlMode.NEUTRAL;
    private double targetPositionDegrees = Double.NaN;
    private double requestedPercent = 0.0;
    private double requestedVolts = 0.0;

    @SuppressWarnings("removal")
    public ConveyorArmSubsystem() {
        leaderMotor = new TalonFX(
            Constants.CanIds.CONVEYOR_ARM_LEADER,
            Constants.ConveyorArm.CAN_BUS
        );
        followerMotor = new TalonFX(
            Constants.CanIds.CONVEYOR_ARM_FOLLOWER,
            Constants.ConveyorArm.CAN_BUS
        );
        absoluteEncoder = new DutyCycleEncoder(Constants.ConveyorArm.ABSOLUTE_ENCODER_DIO_PORT);
        positionController.setTolerance(Constants.ConveyorArm.PRESET_POSITION_TOLERANCE_DEGREES);

        leaderAvailable = configureMotor("leader", leaderMotor, createLeaderConfig());
        followerAvailable = configureMotor("follower", followerMotor, createFollowerConfig());

        if (leaderAvailable) {
            leaderMotor.optimizeBusUtilization(Constants.ConveyorArm.STATUS_UPDATE_HZ);
        }
        if (followerAvailable) {
            followerMotor.optimizeBusUtilization(Constants.ConveyorArm.STATUS_UPDATE_HZ);
            applyFollowerRequest();
        }

        seedLeaderPositionFromAbsoluteEncoder();
        reportHardwareAvailability();
    }

    public void setTelemetryEnabledSuppliers(
        BooleanSupplier calibrationTelemetryEnabledSupplier,
        BooleanSupplier debugTelemetryEnabledSupplier
    ) {
        this.calibrationTelemetryEnabledSupplier =
            calibrationTelemetryEnabledSupplier == null ? () -> false : calibrationTelemetryEnabledSupplier;
        this.debugTelemetryEnabledSupplier =
            debugTelemetryEnabledSupplier == null ? () -> false : debugTelemetryEnabledSupplier;
    }

    public void runManual(double percent) {
        double clampedPercent = MathUtil.clamp(
            percent,
            -Constants.ConveyorArm.MAX_MANUAL_OUTPUT,
            Constants.ConveyorArm.MAX_MANUAL_OUTPUT
        );
        clampedPercent = applyAbsoluteEncoderLimits(clampedPercent);

        if (Math.abs(clampedPercent) < 1e-9) {
            if (controlMode != ArmControlMode.POSITION_HOLD) {
                holdCurrentPosition();
            }
            return;
        }

        controlMode = ArmControlMode.MANUAL;
        targetPositionDegrees = Double.NaN;
        positionController.reset();
        requestedPercent = clampedPercent;
        requestedVolts = clampedPercent * 12.0;

        if (!leaderAvailable) {
            return;
        }

        applyFollowerRequest();
        leaderMotor.setControl(dutyCycleRequest.withOutput(clampedPercent));
    }

    public boolean holdCurrentPosition() {
        if (!isAbsoluteEncoderConnected()) {
            stop();
            return false;
        }
        return setTargetPositionDegrees(getAbsolutePositionDegrees());
    }

    public boolean setTargetPositionDegrees(double targetDegrees) {
        if (!leaderAvailable
                || !isAbsoluteEncoderConnected()
                || !areAbsoluteEncoderLimitsConfigured()
                || !Double.isFinite(targetDegrees)) {
            stop();
            return false;
        }

        targetPositionDegrees = MathUtil.clamp(
            targetDegrees,
            Constants.ConveyorArm.ABSOLUTE_ENCODER_LOW_LIMIT_DEGREES,
            Constants.ConveyorArm.ABSOLUTE_ENCODER_HIGH_LIMIT_DEGREES
        );
        positionController.reset();
        controlMode = ArmControlMode.POSITION_HOLD;
        return true;
    }

    public boolean isAtTargetPosition() {
        return controlMode == ArmControlMode.POSITION_HOLD
            && isAbsoluteEncoderConnected()
            && Math.abs(getPositionErrorDegrees())
                <= Constants.ConveyorArm.PRESET_POSITION_TOLERANCE_DEGREES;
    }

    public double getTargetPositionDegrees() {
        return targetPositionDegrees;
    }

    public double getPositionErrorDegrees() {
        if (!Double.isFinite(targetPositionDegrees) || !isAbsoluteEncoderConnected()) {
            return Double.NaN;
        }
        return targetPositionDegrees - getAbsolutePositionDegrees();
    }

    public void stop() {
        controlMode = ArmControlMode.NEUTRAL;
        targetPositionDegrees = Double.NaN;
        positionController.reset();
        requestedPercent = 0.0;
        requestedVolts = 0.0;

        if (!leaderAvailable) {
            return;
        }

        leaderMotor.setControl(neutralRequest);
        if (followerAvailable) {
            followerMotor.setControl(neutralRequest);
            applyFollowerRequest();
        }
    }

    public double getManualSpeedScale() {
        return Constants.ConveyorArm.MANUAL_SPEED_SCALE_DEFAULT;
    }

    public boolean isAbsoluteEncoderConnected() {
        return absoluteEncoder.isConnected();
    }

    public double getAbsolutePositionRotations() {
        double rotations = absoluteEncoder.get();
        if (Constants.ConveyorArm.ABSOLUTE_ENCODER_INVERTED) {
            rotations = -rotations;
        }
        rotations += Constants.ConveyorArm.ABSOLUTE_ENCODER_OFFSET_ROTATIONS;
        return MathUtil.inputModulus(rotations, 0.0, 1.0);
    }

    public double getAbsolutePositionDegrees() {
        return getAbsolutePositionRotations() * 360.0;
    }

    public boolean areAbsoluteEncoderLimitsConfigured() {
        double lowLimitDegrees = Constants.ConveyorArm.ABSOLUTE_ENCODER_LOW_LIMIT_DEGREES;
        double highLimitDegrees = Constants.ConveyorArm.ABSOLUTE_ENCODER_HIGH_LIMIT_DEGREES;
        return Double.isFinite(lowLimitDegrees)
            && Double.isFinite(highLimitDegrees)
            && lowLimitDegrees >= 0.0
            && highLimitDegrees < 360.0
            && lowLimitDegrees < highLimitDegrees;
    }

    public boolean isAtAbsoluteEncoderLowLimit() {
        return areAbsoluteEncoderLimitsConfigured()
            && getAbsolutePositionDegrees() <= Constants.ConveyorArm.ABSOLUTE_ENCODER_LOW_LIMIT_DEGREES;
    }

    public boolean isAtAbsoluteEncoderHighLimit() {
        return areAbsoluteEncoderLimitsConfigured()
            && getAbsolutePositionDegrees() >= Constants.ConveyorArm.ABSOLUTE_ENCODER_HIGH_LIMIT_DEGREES;
    }

    public double getLeaderPositionRotations() {
        if (!leaderAvailable) {
            return 0.0;
        }
        return leaderMotor.getPosition().getValueAsDouble();
    }

    public double getLeaderVelocityRotationsPerSecond() {
        if (!leaderAvailable) {
            return 0.0;
        }
        return leaderMotor.getVelocity().getValueAsDouble();
    }

    public double getLeaderSupplyCurrentAmps() {
        if (!leaderAvailable) {
            return 0.0;
        }
        return leaderMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getRequestedPercent() {
        return requestedPercent;
    }

    public double getRequestedVolts() {
        return requestedVolts;
    }

    @Override
    public void periodic() {
        if (controlMode == ArmControlMode.POSITION_HOLD) {
            updatePositionHold();
        }

        SmartDashboard.putNumber(DASHBOARD_PREFIX + "AbsolutePositionDeg", getAbsolutePositionDegrees());
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "LeaderPositionRot", getLeaderPositionRotations());
        SmartDashboard.putString(DASHBOARD_PREFIX + "ControlMode", controlMode.name());
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "TargetPositionDeg", targetPositionDegrees);
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "PositionErrorDeg", getPositionErrorDegrees());
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "RequestedPercent", requestedPercent);
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "RequestedVolts", requestedVolts);

        if (!calibrationTelemetryEnabledSupplier.getAsBoolean()
                && !debugTelemetryEnabledSupplier.getAsBoolean()) {
            return;
        }

        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "LeaderAvailable", leaderAvailable);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "FollowerAvailable", followerAvailable);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "AbsoluteEncoderConnected", isAbsoluteEncoderConnected());
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "AbsolutePositionRot", getAbsolutePositionRotations());
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "AbsoluteLimitsConfigured", areAbsoluteEncoderLimitsConfigured());
        SmartDashboard.putNumber(
            DASHBOARD_PREFIX + "AbsoluteLowLimitDeg",
            Constants.ConveyorArm.ABSOLUTE_ENCODER_LOW_LIMIT_DEGREES
        );
        SmartDashboard.putNumber(
            DASHBOARD_PREFIX + "AbsoluteHighLimitDeg",
            Constants.ConveyorArm.ABSOLUTE_ENCODER_HIGH_LIMIT_DEGREES
        );
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "AtLowLimit", isAtAbsoluteEncoderLowLimit());
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "AtHighLimit", isAtAbsoluteEncoderHighLimit());
        if (debugTelemetryEnabledSupplier.getAsBoolean()) {
            SmartDashboard.putNumber(
                DASHBOARD_PREFIX + "LeaderVelocityRps",
                getLeaderVelocityRotationsPerSecond()
            );
            SmartDashboard.putNumber(DASHBOARD_PREFIX + "LeaderSupplyCurrentAmps", getLeaderSupplyCurrentAmps());
        }
    }

    private TalonFXConfiguration createLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.ConveyorArm.LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = Constants.ConveyorArm.BRAKE_MODE
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        applyCurrentLimits(config);
        return config;
    }

    private TalonFXConfiguration createFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.ConveyorArm.BRAKE_MODE
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        applyCurrentLimits(config);
        return config;
    }

    private void applyCurrentLimits(TalonFXConfiguration config) {
        config.CurrentLimits.StatorCurrentLimit = Constants.ConveyorArm.STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.ConveyorArm.SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.ConveyorArm.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.ConveyorArm.SUPPLY_CURRENT_LOWER_TIME_SEC;
    }

    private boolean configureMotor(String role, TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = motor.getConfigurator().apply(config);
        if (!status.isOK()) {
            DriverStation.reportError(
                "Failed to configure conveyor arm "
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

    private void applyFollowerRequest() {
        if (followerAvailable) {
            followerMotor.setControl(followerRequest);
        }
    }

    private void updatePositionHold() {
        if (!leaderAvailable
                || !isAbsoluteEncoderConnected()
                || !areAbsoluteEncoderLimitsConfigured()
                || !Double.isFinite(targetPositionDegrees)) {
            stop();
            return;
        }

        double positionDegrees = getAbsolutePositionDegrees();
        double feedbackVolts = positionController.calculate(positionDegrees, targetPositionDegrees);
        double degreesFromHorizontal =
            positionDegrees - Constants.ConveyorArm.ENCODER_DEGREES_AT_HORIZONTAL;
        double gravityVolts =
            Constants.ConveyorArm.HOLD_KG_VOLTS
                * Math.cos(Math.toRadians(degreesFromHorizontal));

        // PID and gravity terms are expressed in the direction that increases encoder degrees.
        double outputVolts = feedbackVolts + gravityVolts;
        if (!Constants.ConveyorArm.POSITIVE_OUTPUT_INCREASES_ABSOLUTE_ENCODER_DEGREES) {
            outputVolts = -outputVolts;
        }
        outputVolts = MathUtil.clamp(
            outputVolts,
            -Constants.ConveyorArm.MAX_CLOSED_LOOP_VOLTS,
            Constants.ConveyorArm.MAX_CLOSED_LOOP_VOLTS
        );
        outputVolts = applyAbsoluteEncoderLimits(outputVolts);

        requestedVolts = outputVolts;
        requestedPercent = outputVolts / 12.0;
        applyFollowerRequest();
        leaderMotor.setControl(voltageRequest.withOutput(outputVolts));
    }

    private double applyAbsoluteEncoderLimits(double requestedOutput) {
        if (Math.abs(requestedOutput) < 1e-9 || !areAbsoluteEncoderLimitsConfigured()) {
            return requestedOutput;
        }

        // Once real limits are configured, loss of the absolute encoder blocks arm motion.
        if (!isAbsoluteEncoderConnected()) {
            return 0.0;
        }

        boolean outputWouldIncreaseDegrees =
            (requestedOutput > 0.0)
                == Constants.ConveyorArm.POSITIVE_OUTPUT_INCREASES_ABSOLUTE_ENCODER_DEGREES;
        if (outputWouldIncreaseDegrees && isAtAbsoluteEncoderHighLimit()) {
            return 0.0;
        }
        if (!outputWouldIncreaseDegrees && isAtAbsoluteEncoderLowLimit()) {
            return 0.0;
        }
        return requestedOutput;
    }

    private void seedLeaderPositionFromAbsoluteEncoder() {
        if (!leaderAvailable || !isAbsoluteEncoderConnected()) {
            return;
        }

        StatusCode status = leaderMotor.setPosition(getAbsolutePositionRotations());
        if (!status.isOK()) {
            DriverStation.reportWarning(
                "Conveyor arm leader position could not be seeded from the absolute encoder: " + status,
                false
            );
        }
    }

    @SuppressWarnings("removal")
    private void reportHardwareAvailability() {
        if (!leaderAvailable) {
            DriverStation.reportWarning(
                "Conveyor arm leader TalonFX unavailable at startup; conveyor arm output disabled.",
                false
            );
        }
        if (!followerAvailable) {
            DriverStation.reportWarning(
                "Conveyor arm follower TalonFX unavailable at startup; leader will run alone.",
                false
            );
        }
        if (!isAbsoluteEncoderConnected()) {
            DriverStation.reportWarning(
                "Conveyor arm absolute encoder not detected on DIO "
                    + Constants.ConveyorArm.ABSOLUTE_ENCODER_DIO_PORT
                    + ".",
                false
            );
        }
    }
}
