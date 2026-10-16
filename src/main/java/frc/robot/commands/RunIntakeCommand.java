package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class RunIntakeCommand extends Command {

    private final IntakeRollerSubsystem intake;
    private final DoubleSupplier inputSupplier;
    private final DoubleSupplier outputSupplier;
    private final double deadband;

    public RunIntakeCommand(
        IntakeRollerSubsystem intake,
        DoubleSupplier inputSupplier,
        DoubleSupplier outputSupplier,
        double deadband
    ) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;
        this.outputSupplier = outputSupplier;
        this.deadband = deadband;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        double input = MathUtil.clamp(inputSupplier.getAsDouble(), 0.0, 1.0);
        double output = MathUtil.clamp(outputSupplier.getAsDouble(), 0.0, 1.0);

        // RT drives intake in (+), LT drives intake out (-).
        double percent = input - output;
        if (Math.abs(percent) < deadband) {
            percent = 0.0;
        }

        intake.run(percent);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
