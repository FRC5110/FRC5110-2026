package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorArmSubsystem;

public class ManualConveyorArmCommand extends Command {

    private final ConveyorArmSubsystem conveyorArm;
    private final DoubleSupplier rightYSupplier;
    private final DoubleSupplier manualScaleSupplier;
    private final double deadband;

    public ManualConveyorArmCommand(
        ConveyorArmSubsystem conveyorArm,
        DoubleSupplier rightYSupplier,
        double deadband,
        DoubleSupplier manualScaleSupplier
    ) {
        this.conveyorArm = conveyorArm;
        this.rightYSupplier = rightYSupplier;
        this.manualScaleSupplier = manualScaleSupplier;
        this.deadband = deadband;
        addRequirements(conveyorArm);
    }

    @Override
    public void execute() {
        double manualPercent = applyDeadband(rightYSupplier.getAsDouble()) * manualScaleSupplier.getAsDouble();
        conveyorArm.runManual(manualPercent);
    }

    @Override
    public void end(boolean interrupted) {
        conveyorArm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }
}
