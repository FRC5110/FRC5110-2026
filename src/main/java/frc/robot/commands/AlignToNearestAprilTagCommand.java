package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldGeometry;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToNearestAprilTagCommand extends Command {

    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(2.0);
    private static final double TX_TOLERANCE_DEG = 1.0;
    private static final int REQUIRED_STABLE_CYCLES = 5;
    private static final double MAX_ROT_RAD_PER_SEC = 5.0;
    private static final String FRONT_LIMELIGHT = Constants.Vision.Front.LIMELIGHT_NAME;

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier velocityXSupplier;
    private final DoubleSupplier velocityYSupplier;
    private final PIDController poseHeadingPid = new PIDController(5.0, 0.0, 0.1);
    private final PIDController txPid = new PIDController(0.06, 0.0, 0.001);

    private final SwerveRequest.FieldCentric holdAndTurnRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private int stableCycles = 0;
    private Integer targetTagId = null;
    private Pose2d targetTagPose = null;

    public AlignToNearestAprilTagCommand(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier velocityXSupplier,
        DoubleSupplier velocityYSupplier
    ) {
        this.drivetrain = drivetrain;
        this.velocityXSupplier = velocityXSupplier;
        this.velocityYSupplier = velocityYSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        stableCycles = 0;

        poseHeadingPid.enableContinuousInput(-Math.PI, Math.PI);
        poseHeadingPid.reset();
        txPid.reset();

        updateNearestTagTarget();
        SmartDashboard.putString("TagAlign/State", targetTagPose == null ? "NoTarget" : "Aligning");
    }

    @Override
    public void execute() {
        updateNearestTagTarget();

        double vx = velocityXSupplier.getAsDouble();
        double vy = velocityYSupplier.getAsDouble();

        if (targetTagPose == null) {
            drivetrain.setControl(
                holdAndTurnRequest
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(0.0)
            );
            // Realtime dashboard publishing disabled to reduce loop overflows.
            // SmartDashboard.putString("TagAlign/State", "NoTarget");
            // SmartDashboard.putString("TagAlign/Source", "None");
            // SmartDashboard.putNumber("TagAlign/TargetTagId", -1);
            // SmartDashboard.putNumber("TagAlign/OmegaCmd", 0.0);
            // SmartDashboard.putNumber("TagAlign/StableCycles", 0);
            return;
        }

        Pose2d robotPose = drivetrain.getPose();
        double omega;
        double errorDeg;

        if (hasVisibleTargetTag()) {
            double txDeg = LimelightHelpers.getTX(FRONT_LIMELIGHT);
            omega = MathUtil.clamp(
                txPid.calculate(txDeg, 0.0),
                -MAX_ROT_RAD_PER_SEC,
                MAX_ROT_RAD_PER_SEC
            );
            errorDeg = txDeg;

            // Realtime dashboard publishing disabled to reduce loop overflows.
            // SmartDashboard.putString("TagAlign/Source", "LimelightTx");
            // SmartDashboard.putNumber("TagAlign/TxDeg", txDeg);

            if (Math.abs(txDeg) <= TX_TOLERANCE_DEG) {
                stableCycles++;
            } else {
                stableCycles = 0;
            }
        } else {
            double targetHeading = Math.atan2(
                targetTagPose.getY() - robotPose.getY(),
                targetTagPose.getX() - robotPose.getX()
            );
            double currentHeading = robotPose.getRotation().getRadians();
            double headingError = MathUtil.angleModulus(targetHeading - currentHeading);

            omega = MathUtil.clamp(
                poseHeadingPid.calculate(currentHeading, targetHeading),
                -MAX_ROT_RAD_PER_SEC,
                MAX_ROT_RAD_PER_SEC
            );
            errorDeg = Math.toDegrees(headingError);

            // Realtime dashboard publishing disabled to reduce loop overflows.
            // SmartDashboard.putString("TagAlign/Source", "PoseFallback");
            // SmartDashboard.putNumber("TagAlign/TxDeg", Double.NaN);

            if (Math.abs(headingError) <= HEADING_TOLERANCE_RAD) {
                stableCycles++;
            } else {
                stableCycles = 0;
            }
        }

        drivetrain.setControl(
            holdAndTurnRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega)
        );

        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putString("TagAlign/State", "Aligning");
        // SmartDashboard.putNumber("TagAlign/TargetTagId", targetTagId == null ? -1 : targetTagId);
        // SmartDashboard.putNumber("TagAlign/TargetX", targetTagPose.getX());
        // SmartDashboard.putNumber("TagAlign/TargetY", targetTagPose.getY());
        // SmartDashboard.putNumber("TagAlign/ErrorDeg", errorDeg);
        // SmartDashboard.putNumber("TagAlign/VxCmd", vx);
        // SmartDashboard.putNumber("TagAlign/VyCmd", vy);
        // SmartDashboard.putNumber("TagAlign/OmegaCmd", omega);
        // SmartDashboard.putNumber("TagAlign/StableCycles", stableCycles);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            holdAndTurnRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
        SmartDashboard.putString("TagAlign/State", interrupted ? "Interrupted" : "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void updateNearestTagTarget() {
        Optional<Integer> nearestTagId = FieldGeometry.getClosestFieldTag(drivetrain.getPose());
        if (nearestTagId.isEmpty()) {
            targetTagId = null;
            targetTagPose = null;
            return;
        }

        targetTagId = nearestTagId.get();
        targetTagPose = FieldGeometry.FIELD_LAYOUT
            .getTagPose(targetTagId)
            .map(pose -> pose.toPose2d())
            .orElse(null);
    }

    private boolean hasVisibleTargetTag() {
        if (targetTagId == null || !LimelightHelpers.getTV(FRONT_LIMELIGHT)) {
            return false;
        }

        double tidRaw = LimelightHelpers.getFiducialID(FRONT_LIMELIGHT);
        if (!Double.isFinite(tidRaw)) {
            return false;
        }

        return (int) Math.round(tidRaw) == targetTagId;
    }
}
