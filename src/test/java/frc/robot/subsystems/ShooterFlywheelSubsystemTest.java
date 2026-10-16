package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterFlywheelSubsystemTest {

    @Test
    void computeFeedWheelTargetRpmUsesConfiguredRatioMultiplierAndOffset() {
        assertEquals(
            12000.0,
            ShooterFlywheelSubsystem.computeFeedWheelTargetRPM(3000.0, 4.0, 1.0, 0.0),
            1e-9
        );
        assertEquals(
            7700.0,
            ShooterFlywheelSubsystem.computeFeedWheelTargetRPM(1900.0, 4.0, 1.0, 100.0),
            1e-9
        );
        assertEquals(
            0.0,
            ShooterFlywheelSubsystem.computeFeedWheelTargetRPM(Double.NaN, 4.0, 1.0, 0.0),
            1e-9
        );
    }

    @Test
    void computeFixedFeedWheelTargetRpmRunsAtSixThousandWheneverShooterRuns() {
        assertEquals(
            6000.0,
            ShooterFlywheelSubsystem.computeFixedFeedWheelTargetRPM(2900.0, 6000.0),
            1e-9
        );
        assertEquals(
            -6000.0,
            ShooterFlywheelSubsystem.computeFixedFeedWheelTargetRPM(-1500.0, 6000.0),
            1e-9
        );
        assertEquals(
            0.0,
            ShooterFlywheelSubsystem.computeFixedFeedWheelTargetRPM(0.0, 6000.0),
            1e-9
        );
    }

    @Test
    void feedWheelHardFaultConditionRequiresARealCommandAndNearZeroResponse() {
        assertTrue(
            ShooterFlywheelSubsystem.hasFeedWheelHardFaultCondition(4000.0, 0.0, 1000.0, 150.0)
        );
        assertFalse(
            ShooterFlywheelSubsystem.hasFeedWheelHardFaultCondition(4000.0, 500.0, 1000.0, 150.0)
        );
        assertFalse(
            ShooterFlywheelSubsystem.hasFeedWheelHardFaultCondition(500.0, 0.0, 1000.0, 150.0)
        );
    }

    @Test
    void readyToFeedStaysFlywheelBasedButBlocksOnFeedWheelHardFault() {
        assertTrue(ShooterFlywheelSubsystem.computeReadyToFeed(true, false, false));
        assertTrue(ShooterFlywheelSubsystem.computeReadyToFeed(true, true, true));
        assertFalse(ShooterFlywheelSubsystem.computeReadyToFeed(true, true, false));
        assertFalse(ShooterFlywheelSubsystem.computeReadyToFeed(false, true, true));
    }
}
