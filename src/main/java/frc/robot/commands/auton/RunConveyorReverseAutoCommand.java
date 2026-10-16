package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorReverseAutoCommand extends Command {

    private final ConveyorSubsystem conveyor;
    private final double reversePercent;

    public RunConveyorReverseAutoCommand(ConveyorSubsystem conveyor) {
        this(conveyor, Constants.RobotContainer.SHOOT_CONVEYOR_PERCENT);
    }

    public RunConveyorReverseAutoCommand(ConveyorSubsystem conveyor, double percent) {
        this.conveyor = conveyor;
        this.reversePercent = -Math.abs(MathUtil.clamp(percent, -1.0, 1.0));

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoConveyor/State", "Reverse");
    }

    @Override
    public void execute() {
        conveyor.run(reversePercent);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        SmartDashboard.putString("AutoConveyor/State", interrupted ? "ReverseInterrupted" : "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
