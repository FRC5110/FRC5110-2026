package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

@Deprecated
public class SetIntakeArmPositionCommand extends Command {

    public SetIntakeArmPositionCommand(
        IntakeArmSubsystem intakeArm,
        DoubleSupplier targetPositionDegreesSupplier
    ) {
        addRequirements(intakeArm);
    }

    public SetIntakeArmPositionCommand(IntakeArmSubsystem intakeArm, double targetPositionDegrees) {
        this(intakeArm, () -> targetPositionDegrees);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
