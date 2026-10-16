package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorArmSubsystem;

public class MoveConveyorArmToAbsolutePositionCommand extends Command {

    private final ConveyorArmSubsystem conveyorArm;
    private final double targetDegrees;
    private final String targetName;
    private final Timer timer = new Timer();

    private boolean finished;
    private double bestErrorMagnitudeDegrees;

    public MoveConveyorArmToAbsolutePositionCommand(
        ConveyorArmSubsystem conveyorArm,
        double targetDegrees,
        String targetName
    ) {
        this.conveyorArm = conveyorArm;
        this.targetDegrees = targetDegrees;
        this.targetName = targetName;
        addRequirements(conveyorArm);
    }

    @Override
    public void initialize() {
        finished = false;
        timer.restart();

        if (!conveyorArm.areAbsoluteEncoderLimitsConfigured()) {
            fail("absolute-encoder endpoints are not configured");
            return;
        }
        if (!conveyorArm.isAbsoluteEncoderConnected()) {
            fail("absolute encoder is disconnected");
            return;
        }
        if (targetDegrees < Constants.ConveyorArm.ABSOLUTE_ENCODER_LOW_LIMIT_DEGREES
                || targetDegrees > Constants.ConveyorArm.ABSOLUTE_ENCODER_HIGH_LIMIT_DEGREES) {
            fail("target is outside the configured endpoints");
            return;
        }

        if (!conveyorArm.setTargetPositionDegrees(targetDegrees)) {
            fail("position hold could not be enabled");
            return;
        }

        bestErrorMagnitudeDegrees = Math.abs(conveyorArm.getPositionErrorDegrees());
    }

    @Override
    public void execute() {
        if (finished) {
            return;
        }
        if (!conveyorArm.isAbsoluteEncoderConnected()) {
            fail("absolute encoder disconnected during movement");
            return;
        }

        double errorMagnitudeDegrees = Math.abs(conveyorArm.getPositionErrorDegrees());
        if (conveyorArm.isAtTargetPosition()) {
            finished = true;
            return;
        }

        bestErrorMagnitudeDegrees = Math.min(bestErrorMagnitudeDegrees, errorMagnitudeDegrees);
        if (timer.hasElapsed(Constants.ConveyorArm.PRESET_DIRECTION_CHECK_DELAY_SEC)
                && errorMagnitudeDegrees
                    > bestErrorMagnitudeDegrees
                        + Constants.ConveyorArm.PRESET_WRONG_DIRECTION_TOLERANCE_DEGREES) {
            fail("arm moved away from the target; verify the configured output direction");
            return;
        }
        if (timer.hasElapsed(Constants.ConveyorArm.PRESET_TIMEOUT_SEC)) {
            fail("movement timed out");
            return;
        }

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            conveyorArm.holdCurrentPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    private void fail(String reason) {
        finished = true;
        conveyorArm.stop();
        DriverStation.reportWarning(
            "Conveyor arm " + targetName + " preset stopped: " + reason + ".",
            false
        );
    }
}
