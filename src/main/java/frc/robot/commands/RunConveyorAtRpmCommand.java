package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorAtRpmCommand extends Command {

    private final ConveyorSubsystem conveyor;
    private final double targetRpm;

    public RunConveyorAtRpmCommand(ConveyorSubsystem conveyor, double targetRpm) {
        this.conveyor = conveyor;
        this.targetRpm = targetRpm;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setTargetRPM(targetRpm);
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
