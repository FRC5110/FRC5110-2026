package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorPercentCommand extends Command {

    private final ConveyorSubsystem conveyor;
    private final double percent;

    public RunConveyorPercentCommand(ConveyorSubsystem conveyor, double percent) {
        this.conveyor = conveyor;
        this.percent = percent;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.run(percent);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
