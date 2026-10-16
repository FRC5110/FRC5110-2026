package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class RunIntakeReverseAutoCommand extends Command {

    private static final double DEFAULT_REVERSE_PERCENT = 0.70;

    private final IntakeRollerSubsystem intake;
    private final double reversePercent;

    public RunIntakeReverseAutoCommand(IntakeRollerSubsystem intake) {
        this(intake, DEFAULT_REVERSE_PERCENT);
    }

    public RunIntakeReverseAutoCommand(IntakeRollerSubsystem intake, double percent) {
        this.intake = intake;
        this.reversePercent = -Math.abs(MathUtil.clamp(percent, -1.0, 1.0));

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoIntake/State", "Reverse");
    }

    @Override
    public void execute() {
        intake.run(reversePercent);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        SmartDashboard.putString("AutoIntake/State", interrupted ? "ReverseInterrupted" : "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
