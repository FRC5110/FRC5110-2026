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

public class ConveyorSubsystem extends SubsystemBase {

    private static final double MIN_RPM = -6200.0;
    private static final double MAX_RPM = 6200.0;

    private final SparkFlex conveyorMotor;
    private final SparkClosedLoopController conveyorVelocityController;
    private double targetRPM = 0.0;
    private boolean velocityEnabled = false;

    @SuppressWarnings("removal")
    public ConveyorSubsystem() {

        conveyorMotor = new SparkFlex(Constants.CanIds.CONVEYOR, MotorType.kBrushless); // rio CAN ID

        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(true);
        config.idleMode(IdleMode.kBrake);     // Brake helps prevent rollback
        config.smartCurrentLimit(50);        // Safe for Vortex
        config.closedLoop.p(0.0031);
        //config.closedLoop.p(0.00045);
        config.closedLoop.i(0.0);
        config.closedLoop.d(0.0);
        config.closedLoop.velocityFF(0.00020);

        conveyorMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        conveyorVelocityController = conveyorMotor.getClosedLoopController();
    }

    // + forward, - reverse
    public void run(double percent) {
        velocityEnabled = false;
        targetRPM = 0.0;
        conveyorMotor.set(percent);
    }

    @SuppressWarnings("removal")
    public void setTargetRPM(double rpm) {
        targetRPM = MathUtil.clamp(rpm, MIN_RPM, MAX_RPM);

        if (Math.abs(targetRPM) < 1e-3) {
            stop();
            return;
        }

        velocityEnabled = true;
        conveyorVelocityController.setReference(
            targetRPM,
            SparkFlex.ControlType.kVelocity
        );
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return conveyorMotor.getEncoder().getVelocity();
    }

    public void stop() {
        velocityEnabled = false;
        targetRPM = 0.0;
        conveyorMotor.set(0);
    }

    public double getPositionRotations() {
        return conveyorMotor.getEncoder().getPosition();
    }

    public double getPositionDegrees() {
        return getPositionRotations() * 360.0;
    }

    public double getSpeedRPM() {
        return conveyorMotor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putNumber("Aux/Conveyor/TargetRPM", targetRPM);
        // SmartDashboard.putNumber("Aux/Conveyor/CurrentRPM", getCurrentRPM());
        // SmartDashboard.putBoolean("Aux/Conveyor/VelocityEnabled", velocityEnabled);
        // SmartDashboard.putNumber("Aux/Conveyor/PositionRot", getPositionRotations());
        // SmartDashboard.putNumber("Aux/Conveyor/PositionDeg", getPositionDegrees());
        // SmartDashboard.putNumber("Aux/Conveyor/SpeedRPM", getSpeedRPM());
    }
}
