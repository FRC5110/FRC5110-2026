package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorForwardAutoCommand extends Command {

    private final ConveyorSubsystem conveyor;
    private final double forwardPercent;

    public RunConveyorForwardAutoCommand(ConveyorSubsystem conveyor) {
        this(conveyor, Constants.RobotContainer.SHOOT_CONVEYOR_PERCENT);
    }

    public RunConveyorForwardAutoCommand(ConveyorSubsystem conveyor, double percent) {
        this.conveyor = conveyor;
        this.forwardPercent = Math.abs(MathUtil.clamp(percent, -1.0, 1.0));

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoConveyor/State", "Forward");
    }

    @Override
    public void execute() {
        conveyor.run(forwardPercent);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        SmartDashboard.putString("AutoConveyor/State", interrupted ? "ForwardInterrupted" : "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
