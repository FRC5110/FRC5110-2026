package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;

import java.util.Optional;

public final class FieldGeometry {

    private static final int[] BLUE_FUEL_HUB_FACE_TAG_IDS = {25, 26};
    private static final int[] RED_FUEL_HUB_FACE_TAG_IDS = {9, 10};
    private static final int[] BLUE_FUEL_HUB_ALL_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
    private static final int[] RED_FUEL_HUB_ALL_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};

    private FieldGeometry() {}

    /* ===================================================== */
    /* ================= FIELD LAYOUT ====================== */
    /* ===================================================== */

    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(
                    AprilTagFields.k2026RebuiltAndymark
            );

    /*
        FIELD COORDINATE SYSTEM (WPILib Standard)

        Origin (0,0):
            Blue alliance driver station LOWER-LEFT corner
            (when viewed in simulation)

        +X:
            Toward Red alliance wall

        +Y:
            Toward left side of field from Blue perspective

        0° Rotation:
            Facing +X direction

        Rotation:
            Counterclockwise positive
    */

    /* ===================================================== */
    /* ================= ALLIANCE CHECK ==================== */
    /* ===================================================== */

    public static boolean isBlue() {
        return AllianceUtil.isBlue();
    }

    /* ===================================================== */
    /* ============ ROBOT ORIENTATION CONFIG =============== */
    /* ===================================================== */

    /*
        Change these if robot orientation changes.

        SHOOTER_FORWARD:
            Robot front faces target

        INTAKE_BACKWARD:
            Robot back faces target
     */

    public static final Rotation2d SHOOTER_FORWARD =
            Rotation2d.fromDegrees(0);

    public static final Rotation2d INTAKE_BACKWARD =
            Rotation2d.fromDegrees(180);

    /* ===================================================== */
    /* ================= OUTPOST SETTINGS ================== */
    /* ===================================================== */

    // Distance forward from tag to stop
    public static final double OUTPOST_FORWARD_OFFSET = 0.85;

    // Left/right shift relative to tag
    public static final double OUTPOST_LATERAL_OFFSET = 0.0;

    /* ===================================================== */
    /* ================= FUEL HUB SETTINGS ================= */
    /* ===================================================== */

    // Distance away from hub center
    public static final double HUB_DISTANCE_OFFSET = 1.1;

    /* ===================================================== */
    /* ======== CLOSEST ALLIANCE TAG SELECTION ============ */
    /* ===================================================== */

    public static Optional<Integer> getClosestAllianceTag(
            Pose2d robotPose
    ) {
        boolean blue = isBlue();
        double bestDistance = Double.MAX_VALUE;
        Integer bestTag = null;

        for (int id = 1; id <= 32; id++) {

            // Alliance filtering:
            // 1–16 = Red
            // 17–32 = Blue
            if (blue && id <= 16) continue;
            if (!blue && id >= 17) continue;

            var tagOpt = FIELD_LAYOUT.getTagPose(id);
            if (tagOpt.isEmpty()) continue;

            Pose2d tagPose = tagOpt.get().toPose2d();

            double distance =
                    robotPose.getTranslation()
                            .getDistance(tagPose.getTranslation());

            if (distance < bestDistance) {
                bestDistance = distance;
                bestTag = id;
            }
        }

        return Optional.ofNullable(bestTag);
    }

    public static Optional<Integer> getClosestFieldTag(
            Pose2d robotPose
    ) {
        double bestDistance = Double.MAX_VALUE;
        Integer bestTag = null;

        for (int id = 1; id <= 32; id++) {
            var tagOpt = FIELD_LAYOUT.getTagPose(id);
            if (tagOpt.isEmpty()) continue;

            Pose2d tagPose = tagOpt.get().toPose2d();

            double distance =
                    robotPose.getTranslation()
                            .getDistance(tagPose.getTranslation());

            if (distance < bestDistance) {
                bestDistance = distance;
                bestTag = id;
            }
        }

        return Optional.ofNullable(bestTag);
    }

    /* ===================================================== */
    /* ================= OUTPOST TARGET ==================== */
    /* ===================================================== */

    public static Optional<Pose2d> getOutpostTargetPose(
            Pose2d robotPose
    ) {

        Optional<Integer> tagId =
                getClosestAllianceTag(robotPose);

        if (tagId.isEmpty()) return Optional.empty();

        var tagOpt = FIELD_LAYOUT.getTagPose(tagId.get());
        if (tagOpt.isEmpty()) return Optional.empty();

        Pose2d tagPose = tagOpt.get().toPose2d();

        // Apply translation offset from tag
        Pose2d translated =
                tagPose.transformBy(
                        new Transform2d(
                                new Translation2d(
                                        OUTPOST_FORWARD_OFFSET,
                                        OUTPOST_LATERAL_OFFSET
                                ),
                                new Rotation2d()
                        )
                );

        // Face OUTPOST using intake (robot back toward tag)
        Rotation2d heading =
                tagPose.getRotation()
                        .rotateBy(INTAKE_BACKWARD);

        return Optional.of(
                new Pose2d(
                        translated.getTranslation(),
                        heading
                )
        );
    }

    /* ===================================================== */
    /* ================= FUEL HUB CENTER =================== */
    /* ===================================================== */

    public static int[] getBlueFuelHubTagIds() {
        return BLUE_FUEL_HUB_FACE_TAG_IDS.clone();
    }

    public static int[] getRedFuelHubTagIds() {
        return RED_FUEL_HUB_FACE_TAG_IDS.clone();
    }

    public static int[] getFuelHubTagIds() {
        return isBlue()
                ? getBlueFuelHubTagIds()
                : getRedFuelHubTagIds();
    }

    public static int[] getBlueFuelHubAllTagIds() {
        return BLUE_FUEL_HUB_ALL_TAG_IDS.clone();
    }

    public static int[] getRedFuelHubAllTagIds() {
        return RED_FUEL_HUB_ALL_TAG_IDS.clone();
    }

    public static int[] getFuelHubAllTagIds() {
        return isBlue()
                ? getBlueFuelHubAllTagIds()
                : getRedFuelHubAllTagIds();
    }

    public static int[][] getFuelHubAllTagIdCandidates() {
        return new int[][] {
            getFuelHubAllTagIds(),
            isBlue() ? getRedFuelHubAllTagIds() : getBlueFuelHubAllTagIds()
        };
    }

    public static int[] getAllFuelHubTagIds() {
        int[] blueIds = getBlueFuelHubAllTagIds();
        int[] redIds = getRedFuelHubAllTagIds();
        int[] allIds = new int[blueIds.length + redIds.length];
        System.arraycopy(blueIds, 0, allIds, 0, blueIds.length);
        System.arraycopy(redIds, 0, allIds, blueIds.length, redIds.length);
        return allIds;
    }

    public static int[][] getFuelHubTagIdCandidates() {
        return new int[][] {
            getFuelHubTagIds(),
            isBlue() ? getRedFuelHubTagIds() : getBlueFuelHubTagIds()
        };
    }

    public static Translation2d getFuelHubCenter() {
        return getFuelHubCenter(getFuelHubTagIds());
    }

    public static Translation2d getFuelHubTrueCenter() {
        return getFuelHubCenter(getFuelHubAllTagIds());
    }

    public static Translation2d getFuelHubCenter(int[] hubTags) {
        double totalX = 0.0;
        double totalY = 0.0;
        int count = 0;

        for (int tagId : hubTags) {
            var tagOpt = FIELD_LAYOUT.getTagPose(tagId);
            if (tagOpt.isEmpty()) continue;

            Translation2d tagTranslation = tagOpt.get().toPose2d().getTranslation();
            totalX += tagTranslation.getX();
            totalY += tagTranslation.getY();
            count++;
        }

        if (count == 0) {
            return new Translation2d();
        }

        return new Translation2d(totalX / count, totalY / count);
    }

    public static Optional<Translation2d> getFuelHubCenterOffsetFromTagFrame(int tagId) {
        Translation2d hubCenter = getFuelHubCenterForTag(tagId);
        if (hubCenter == null) {
            return Optional.empty();
        }

        var tagOpt = FIELD_LAYOUT.getTagPose(tagId);
        if (tagOpt.isEmpty()) {
            return Optional.empty();
        }

        Pose2d tagPose = tagOpt.get().toPose2d();
        Pose2d hubCenterPose = new Pose2d(hubCenter, new Rotation2d());
        return Optional.of(hubCenterPose.relativeTo(tagPose).getTranslation());
    }

    /* ===================================================== */
    /* ================= FUEL HUB TARGET =================== */
    /* ===================================================== */

    public static Pose2d getFuelHubTargetPose(
            Pose2d robotPose
    ) {

        Translation2d hubCenter = getFuelHubTrueCenter();

        Translation2d direction =
                robotPose.getTranslation()
                        .minus(hubCenter);

        double distance = direction.getNorm();

        Translation2d unit =
                distance > 1e-6
                        ? direction.div(distance)
                        : new Translation2d();

        // Face hub using shooter/front
        Rotation2d facing =
                new Rotation2d(
                        Math.atan2(
                                hubCenter.getY() - robotPose.getY(),
                                hubCenter.getX() - robotPose.getX()
                        )
                ).rotateBy(SHOOTER_FORWARD);

        Translation2d stopPoint =
                hubCenter.plus(
                        unit.times(HUB_DISTANCE_OFFSET)
                );

        return new Pose2d(stopPoint, facing);
    }

    private static Translation2d getFuelHubCenterForTag(int tagId) {
        if (contains(BLUE_FUEL_HUB_ALL_TAG_IDS, tagId)) {
            return getFuelHubCenter(BLUE_FUEL_HUB_ALL_TAG_IDS);
        }
        if (contains(RED_FUEL_HUB_ALL_TAG_IDS, tagId)) {
            return getFuelHubCenter(RED_FUEL_HUB_ALL_TAG_IDS);
        }
        return null;
    }

    private static boolean contains(int[] values, int target) {
        for (int value : values) {
            if (value == target) {
                return true;
            }
        }
        return false;
    }
}
