package frc.robot.commands.auton;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ShooterFeedGate;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

@Deprecated
public class ShootThreeThenLowerArmAndFeedCommand extends Command {

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier rpmSupplier;
    private final ShooterFeedGate feedGate = new ShooterFeedGate();

    public ShootThreeThenLowerArmAndFeedCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        IntakeArmSubsystem intakeArm,
        DoubleSupplier rpmSupplier
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpmSupplier = rpmSupplier;

        addRequirements(shooter, conveyor, intakeArm);
    }

    @Override
    public void initialize() {
        feedGate.reset();
        conveyor.stop();
    }

    @Override
    public void execute() {
        shooter.setTargetRPM(rpmSupplier.getAsDouble());

        double feedPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_FEED_POWER_DEFAULT, -1.0, 1.0);
        double holdPower = MathUtil.clamp(Constants.ShooterTune.CONVEYOR_HOLD_POWER_DEFAULT, -1.0, 1.0);
        boolean canFeed = feedGate.shouldFeed(shooter);
        conveyor.run(canFeed ? feedPower : holdPower);
    }

    @Override
    public void end(boolean interrupted) {
        feedGate.reset();
        conveyor.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
