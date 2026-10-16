package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class RunIntakeForwardAutoCommand extends Command {

    private static final double DEFAULT_FORWARD_PERCENT = 0.70;

    private final IntakeRollerSubsystem intake;
    private final double forwardPercent;

    public RunIntakeForwardAutoCommand(IntakeRollerSubsystem intake) {
        this(intake, DEFAULT_FORWARD_PERCENT);
    }

    public RunIntakeForwardAutoCommand(IntakeRollerSubsystem intake, double percent) {
        this.intake = intake;
        this.forwardPercent = Math.abs(MathUtil.clamp(percent, -1.0, 1.0));

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoIntake/State", "Forward");
    }

    @Override
    public void execute() {
        intake.run(forwardPercent);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        SmartDashboard.putString("AutoIntake/State", interrupted ? "ForwardInterrupted" : "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
