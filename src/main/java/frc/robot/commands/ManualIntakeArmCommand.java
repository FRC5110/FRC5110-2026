package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.IntakeArmSubsystem;

@Deprecated
public class ManualIntakeArmCommand extends ManualConveyorArmCommand {

    public ManualIntakeArmCommand(
        IntakeArmSubsystem intakeArm,
        DoubleSupplier rightYSupplier,
        double deadband,
        DoubleSupplier manualScaleSupplier
    ) {
        super(intakeArm, rightYSupplier, deadband, manualScaleSupplier);
    }

    public ManualIntakeArmCommand(
        IntakeArmSubsystem intakeArm,
        DoubleSupplier rightYSupplier,
        double deadband,
        double manualScale
    ) {
        super(intakeArm, rightYSupplier, deadband, () -> manualScale);
    }
}
