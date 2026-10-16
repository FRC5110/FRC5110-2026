package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

class HubTagVisionTest {

    private static final double EPSILON = 1e-9;
    private static final AtomicInteger TEST_ID = new AtomicInteger();

    @Test
    void estimatePreferredFuelHubCenterRobotSpaceRealtimeReturnsValidEstimateForRawFiducials() {
        String limelightName = uniqueLimelightName();
        int[] hubTagIds = FieldGeometry.getFuelHubAllTagIdCandidates()[0];
        int tagId = hubTagIds[0];
        Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);

        setRawFiducials(
            limelightName,
            new double[] {tagId, 0.0, 0.0, 5.0, 1.0, 1.0, 0.0}
        );

        HubTagVision.HubCenterEstimate estimate =
            HubTagVision.estimatePreferredFuelHubCenterRobotSpaceRealtime(limelightName, robotPose);

        Translation2d expectedRobotToTag = new Translation2d(1.0, Rotation2d.kZero);
        Translation2d expectedTagToHub =
            FieldGeometry.getFuelHubCenter(hubTagIds).minus(
                FieldGeometry.FIELD_LAYOUT.getTagPose(tagId).orElseThrow().toPose2d().getTranslation()
            );
        Translation2d expectedRobotToHub = expectedRobotToTag.plus(expectedTagToHub);

        assertTrue(estimate.isValid());
        assertEquals(expectedRobotToHub.getX(), estimate.robotSpaceX(), EPSILON);
        assertEquals(expectedRobotToHub.getY(), estimate.robotSpaceY(), EPSILON);
        assertEquals(expectedRobotToHub.getNorm(), estimate.distanceMeters(), EPSILON);
        assertEquals(1.0, estimate.averageVisibleTagDistanceMeters(), EPSILON);
        assertEquals(1, estimate.visibleHubTagCount());
        assertEquals(0.0, estimate.estimateSpreadMeters(), EPSILON);
    }

    @Test
    void estimatePreferredFuelHubCenterRobotSpaceRealtimeReturnsEmptyWhenRawFiducialsAreMissing() {
        String limelightName = uniqueLimelightName();

        setRawFiducials(limelightName, new double[0]);

        HubTagVision.HubCenterEstimate estimate =
            HubTagVision.estimatePreferredFuelHubCenterRobotSpaceRealtime(
                limelightName,
                new Pose2d()
            );

        assertFalse(estimate.isValid());
        assertEquals(0, estimate.visibleHubTagCount());
    }

    @Test
    void estimatePreferredFuelHubCenterRobotSpaceRealtimeReturnsEmptyForMalformedRawFiducials() {
        String limelightName = uniqueLimelightName();

        setRawFiducials(limelightName, new double[] {25.0, 0.0, 0.0});

        HubTagVision.HubCenterEstimate estimate =
            HubTagVision.estimatePreferredFuelHubCenterRobotSpaceRealtime(
                limelightName,
                new Pose2d()
            );

        assertFalse(estimate.isValid());
        assertEquals(0, estimate.visibleHubTagCount());
    }

    private static String uniqueLimelightName() {
        return "limelight-hubtag-test-" + TEST_ID.incrementAndGet();
    }

    private static void setRawFiducials(String limelightName, double[] rawFiducials) {
        NetworkTableInstance.getDefault()
            .getTable(limelightName)
            .getEntry("rawfiducials")
            .setDoubleArray(rawFiducials);
        NetworkTableInstance.getDefault().flush();
    }
}
