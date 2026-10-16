package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTableInstance;

class LimelightHelpersTest {

    private static final AtomicInteger TEST_ID = new AtomicInteger();

    @Test
    void tryWarmLatestResultsJsonReturnsFalseForBlankJson() {
        String limelightName = uniqueLimelightName();
        setJson(limelightName, "   ");

        assertFalse(LimelightHelpers.tryWarmLatestResultsJson(limelightName));
    }

    @Test
    void tryWarmLatestResultsJsonReturnsTrueForMinimalJsonAndKeepsResultsReadable() {
        String limelightName = uniqueLimelightName();
        setJson(limelightName, "{}");

        assertTrue(LimelightHelpers.tryWarmLatestResultsJson(limelightName));

        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

        assertNotNull(results);
        assertNotNull(results.targets_Fiducials);
        assertEquals(0, results.targets_Fiducials.length);
        assertEquals("", results.pipelineType);
    }

    private static String uniqueLimelightName() {
        return "limelight-test-" + TEST_ID.incrementAndGet();
    }

    private static void setJson(String limelightName, String json) {
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("json").setString(json);
        NetworkTableInstance.getDefault().flush();
    }
}
