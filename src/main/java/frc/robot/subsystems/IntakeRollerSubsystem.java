package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {

    private static final double MIN_RPM = -6200.0;
    private static final double MAX_RPM = 6200.0;

    private final SparkFlex intakeMotor;
    private final SparkClosedLoopController intakeVelocityController;
    private double targetRPM = 0.0;
    private boolean velocityEnabled = false;

    @SuppressWarnings("removal")
    public IntakeRollerSubsystem() {
        intakeMotor = new SparkFlex(Constants.CanIds.INTAKE_ROLLER, MotorType.kBrushless); // rio CAN

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(50);
        config.closedLoop.p(0.00045);
        config.closedLoop.i(0.0);
        config.closedLoop.d(0.0);
        config.closedLoop.velocityFF(0.00020);

        intakeMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        intakeVelocityController = intakeMotor.getClosedLoopController();
    }

    // + forward, - reverse
    public void run(double percent) {
        velocityEnabled = false;
        targetRPM = 0.0;
        intakeMotor.set(percent);
    }

    @SuppressWarnings("removal")
    public void setTargetRPM(double rpm) {
        targetRPM = MathUtil.clamp(rpm, MIN_RPM, MAX_RPM);

        if (Math.abs(targetRPM) < 1e-3) {
            stop();
            return;
        }

        velocityEnabled = true;
        intakeVelocityController.setReference(
            targetRPM,
            SparkFlex.ControlType.kVelocity
        );
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return intakeMotor.getEncoder().getVelocity();
    }

    public void stop() {
        velocityEnabled = false;
        targetRPM = 0.0;
        intakeMotor.set(0);
    }

    public double getPositionRotations() {
        return intakeMotor.getEncoder().getPosition();
    }

    public double getPositionDegrees() {
        return getPositionRotations() * 360.0;
    }

    public double getSpeedRPM() {
        return intakeMotor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putNumber("Aux/IntakeRoller/TargetRPM", targetRPM);
        // SmartDashboard.putNumber("Aux/IntakeRoller/CurrentRPM", getCurrentRPM());
        // SmartDashboard.putBoolean("Aux/IntakeRoller/VelocityEnabled", velocityEnabled);
        // SmartDashboard.putNumber("Aux/IntakeRoller/PositionRot", getPositionRotations());
        // SmartDashboard.putNumber("Aux/IntakeRoller/PositionDeg", getPositionDegrees());
        // SmartDashboard.putNumber("Aux/IntakeRoller/SpeedRPM", getSpeedRPM());
    }
}
