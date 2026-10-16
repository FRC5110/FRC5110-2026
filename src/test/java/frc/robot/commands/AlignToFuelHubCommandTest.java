package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers;

class AlignToFuelHubCommandTest {

    private static final double EPSILON = 1e-9;

    @Test
    void calculateTargetHeadingRadPointsAtHubCenter() {
        Translation2d hubCenter = new Translation2d(5.0, 3.0);

        assertAngleEquals(
            0.0,
            AlignToFuelHubCommand.calculateTargetHeadingRad(
                new Pose2d(4.0, 3.0, Rotation2d.kZero),
                hubCenter
            )
        );
        assertAngleEquals(
            Math.PI / 2.0,
            AlignToFuelHubCommand.calculateTargetHeadingRad(
                new Pose2d(5.0, 2.0, Rotation2d.kZero),
                hubCenter
            )
        );
        assertAngleEquals(
            Math.PI,
            AlignToFuelHubCommand.calculateTargetHeadingRad(
                new Pose2d(6.0, 3.0, Rotation2d.kZero),
                hubCenter
            )
        );
    }

    @Test
    void isUsablePoseEstimateRequiresTwoTagsAndFinitePose() {
        assertTrue(
            AlignToFuelHubCommand.isUsablePoseEstimate(
                poseEstimate(new Pose2d(2.0, 1.0, Rotation2d.kZero), 2)
            )
        );
        assertFalse(
            AlignToFuelHubCommand.isUsablePoseEstimate(
                poseEstimate(new Pose2d(2.0, 1.0, Rotation2d.kZero), 1)
            )
        );
        assertFalse(
            AlignToFuelHubCommand.isUsablePoseEstimate(
                poseEstimate(new Pose2d(Double.NaN, 1.0, Rotation2d.kZero), 2)
            )
        );
    }

    @Test
    void selectPoseAimTargetUsesFreshPoseWhenEstimateIsUsable() {
        Translation2d hubCenter = new Translation2d(4.0, 3.0);
        LimelightHelpers.PoseEstimate poseEstimate =
            poseEstimate(new Pose2d(3.0, 3.0, Rotation2d.kZero), 2);

        AlignToFuelHubCommand.PoseAimTarget result = AlignToFuelHubCommand.selectPoseAimTarget(
            poseEstimate,
            hubCenter,
            10.0,
            Double.NaN,
            Double.NEGATIVE_INFINITY,
            Double.NaN,
            0.10
        );

        assertTrue(result.hasFreshPose());
        assertFalse(result.usingLatchedHeading());
        assertAngleEquals(0.0, result.targetHeadingRad());
        assertEquals(1.0, result.distanceMeters(), EPSILON);
        assertEquals(2, result.tagCount());
        assertEquals("LimelightPoseHubHeading", result.alignSource());
    }

    @Test
    void selectPoseAimTargetLatchesRecentHeadingWhenPoseDropsOut() {
        AlignToFuelHubCommand.PoseAimTarget result = AlignToFuelHubCommand.selectPoseAimTarget(
            poseEstimate(new Pose2d(1.0, 2.0, Rotation2d.kZero), 1),
            new Translation2d(4.0, 3.0),
            10.0,
            1.23,
            9.95,
            3.4,
            0.10
        );

        assertFalse(result.hasFreshPose());
        assertTrue(result.usingLatchedHeading());
        assertEquals(1.23, result.targetHeadingRad(), EPSILON);
        assertEquals(3.4, result.distanceMeters(), EPSILON);
        assertEquals("LimelightPoseHubHeadingLatched", result.alignSource());
    }

    @Test
    void selectPoseAimTargetDropsExpiredLatchWhenPoseIsNotUsable() {
        AlignToFuelHubCommand.PoseAimTarget result = AlignToFuelHubCommand.selectPoseAimTarget(
            poseEstimate(new Pose2d(1.0, 2.0, Rotation2d.kZero), 1),
            new Translation2d(4.0, 3.0),
            10.0,
            1.23,
            9.80,
            3.4,
            0.10
        );

        assertFalse(result.hasFreshPose());
        assertFalse(result.usingLatchedHeading());
        assertFalse(result.hasTargetHeading());
        assertEquals("NoLimelightHubPose", result.alignSource());
    }

    private static LimelightHelpers.PoseEstimate poseEstimate(Pose2d pose, int tagCount) {
        return new LimelightHelpers.PoseEstimate(
            pose,
            0.0,
            0.0,
            tagCount,
            0.0,
            0.0,
            0.0,
            new LimelightHelpers.RawFiducial[0],
            false
        );
    }

    private static void assertAngleEquals(double expected, double actual) {
        assertEquals(0.0, MathUtil.angleModulus(actual - expected), EPSILON);
    }
}
