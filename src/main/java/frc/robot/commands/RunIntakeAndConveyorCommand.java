package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class RunIntakeAndConveyorCommand extends Command {

    private final IntakeRollerSubsystem intake;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier intakePercentSupplier;
    private final DoubleSupplier conveyorPercentSupplier;

    public RunIntakeAndConveyorCommand(
        IntakeRollerSubsystem intake,
        ConveyorSubsystem conveyor,
        DoubleSupplier intakePercentSupplier,
        DoubleSupplier conveyorPercentSupplier
    ) {
        this.intake = intake;
        this.conveyor = conveyor;
        this.intakePercentSupplier = intakePercentSupplier;
        this.conveyorPercentSupplier = conveyorPercentSupplier;

        addRequirements(intake, conveyor);
    }

    @Override
    public void execute() {
        double intakePercent = MathUtil.clamp(intakePercentSupplier.getAsDouble(), -1.0, 1.0);
        double conveyorPercent = MathUtil.clamp(conveyorPercentSupplier.getAsDouble(), -1.0, 1.0);

        intake.run(intakePercent);
        conveyor.run(conveyorPercent);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
