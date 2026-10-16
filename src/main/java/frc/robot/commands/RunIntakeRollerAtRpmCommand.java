package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class RunIntakeRollerAtRpmCommand extends Command {

    private final IntakeRollerSubsystem intakeRoller;
    private final double targetRpm;

    public RunIntakeRollerAtRpmCommand(IntakeRollerSubsystem intakeRoller, double targetRpm) {
        this.intakeRoller = intakeRoller;
        this.targetRpm = targetRpm;
        addRequirements(intakeRoller);
    }

    @Override
    public void execute() {
        intakeRoller.setTargetRPM(targetRpm);
    }

    @Override
    public void end(boolean interrupted) {
        intakeRoller.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
