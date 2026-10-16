package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CalibrateHubDistanceCommand extends Command {

    private final Runnable updateAction;
    private final Runnable clearLedAction;

    public CalibrateHubDistanceCommand(Runnable updateAction) {
        this(updateAction, () -> {});
    }

    public CalibrateHubDistanceCommand(Runnable updateAction, Runnable clearLedAction) {
        this.updateAction = updateAction;
        this.clearLedAction = clearLedAction;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoShoot/CalibrationState", "Running");
    }

    @Override
    public void execute() {
        updateAction.run();
    }

    @Override
    public void end(boolean interrupted) {
        clearLedAction.run();
        SmartDashboard.putString(
            "AutoShoot/CalibrationState",
            interrupted ? "Interrupted" : "Idle"
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
