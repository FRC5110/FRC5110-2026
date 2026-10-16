package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldGeometry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignFieldCentric extends Command {

    public enum AlignMode {
        FUEL_HUB,
        OUTPOST
    }

    private static final double MAX_VEL_METERS_PER_SEC = 1.8;
    private static final double MAX_ROT_RAD_PER_SEC = 2.2;
    private static final int REQUIRED_STABLE_CYCLES = 5;

    private final CommandSwerveDrivetrain drivetrain;
    private final AlignMode mode;

    private final SwerveRequest.FieldCentric request =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final ProfiledPIDController xPid =
        new ProfiledPIDController(
            1.6,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_VEL_METERS_PER_SEC, 3.0)
        );
    private final ProfiledPIDController yPid =
        new ProfiledPIDController(
            1.6,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_VEL_METERS_PER_SEC, 3.0)
        );
    private final ProfiledPIDController rotPid =
        new ProfiledPIDController(
            4.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_ROT_RAD_PER_SEC, 6.0)
        );

    private Pose2d targetPose;
    private boolean validTarget = false;
    private int stableCycles = 0;

    public AutoAlignFieldCentric(CommandSwerveDrivetrain drivetrain, AlignMode mode) {
        this.drivetrain = drivetrain;
        this.mode = mode;

        xPid.setTolerance(0.07);
        yPid.setTolerance(0.07);
        rotPid.setTolerance(Math.toRadians(8.0));
        rotPid.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getPose();

        targetPose = selectTargetPose(currentPose).orElse(null);
        validTarget = targetPose != null;
        stableCycles = 0;

        xPid.reset(currentPose.getX());
        yPid.reset(currentPose.getY());
        rotPid.reset(currentPose.getRotation().getRadians());

        SmartDashboard.putString("AutoAlignFC/Mode", mode.name());
        SmartDashboard.putBoolean("AutoAlignFC/ValidTarget", validTarget);

        if (!validTarget) {
            SmartDashboard.putString("AutoAlignFC/State", "NoTarget");
            return;
        }

        SmartDashboard.putString("AutoAlignFC/State", "Aligning");
        SmartDashboard.putNumber("AutoAlignFC/TargetX", targetPose.getX());
        SmartDashboard.putNumber("AutoAlignFC/TargetY", targetPose.getY());
        SmartDashboard.putNumber("AutoAlignFC/TargetHeadingDeg", targetPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        if (!validTarget) {
            drivetrain.stop();
            return;
        }

        Pose2d currentPose = drivetrain.getPose();

        double vx = MathUtil.clamp(
            xPid.calculate(currentPose.getX(), targetPose.getX()),
            -MAX_VEL_METERS_PER_SEC,
            MAX_VEL_METERS_PER_SEC
        );
        double vy = MathUtil.clamp(
            yPid.calculate(currentPose.getY(), targetPose.getY()),
            -MAX_VEL_METERS_PER_SEC,
            MAX_VEL_METERS_PER_SEC
        );
        double omega = MathUtil.clamp(
            rotPid.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            ),
            -MAX_ROT_RAD_PER_SEC,
            MAX_ROT_RAD_PER_SEC
        );

        drivetrain.setControl(
            request
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega)
        );

        if (xPid.atGoal() && yPid.atGoal() && rotPid.atGoal()) {
            stableCycles++;
        } else {
            stableCycles = 0;
        }

        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putString("AutoAlignFC/State", "Aligning");
        // SmartDashboard.putNumber("AutoAlignFC/ErrorX", targetPose.getX() - currentPose.getX());
        // SmartDashboard.putNumber("AutoAlignFC/ErrorY", targetPose.getY() - currentPose.getY());
        // SmartDashboard.putNumber(
        //     "AutoAlignFC/ErrorHeadingDeg",
        //     Math.toDegrees(
        //         MathUtil.angleModulus(
        //             targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()
        //         )
        //     )
        // );
        // SmartDashboard.putNumber("AutoAlignFC/VxCmd", vx);
        // SmartDashboard.putNumber("AutoAlignFC/VyCmd", vy);
        // SmartDashboard.putNumber("AutoAlignFC/OmegaCmd", omega);
        // SmartDashboard.putNumber("AutoAlignFC/StableCycles", stableCycles);
    }

    @Override
    public boolean isFinished() {
        return !validTarget || stableCycles >= REQUIRED_STABLE_CYCLES;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        SmartDashboard.putString(
            "AutoAlignFC/State",
            interrupted ? "Interrupted" : (validTarget ? "Aligned" : "NoTarget")
        );
    }

    private Optional<Pose2d> selectTargetPose(Pose2d currentPose) {
        return switch (mode) {
            case FUEL_HUB -> Optional.of(FieldGeometry.getFuelHubTargetPose(currentPose));
            case OUTPOST -> FieldGeometry.getOutpostTargetPose(currentPose);
        };
    }
}
