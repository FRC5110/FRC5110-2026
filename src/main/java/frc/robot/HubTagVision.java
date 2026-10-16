package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class HubTagVision {

    public record HubCenterEstimate(
        Translation2d robotToHubCenter,
        double averageVisibleTagDistanceMeters,
        int visibleHubTagCount,
        double estimateSpreadMeters
    ) {
        public static final HubCenterEstimate EMPTY =
            new HubCenterEstimate(null, Double.NaN, 0, Double.NaN);

        public boolean isValid() {
            return robotToHubCenter != null
                && Double.isFinite(robotToHubCenter.getX())
                && Double.isFinite(robotToHubCenter.getY())
                && robotToHubCenter.getNorm() > 1e-6;
        }

        public double distanceMeters() {
            return isValid() ? robotToHubCenter.getNorm() : Double.NaN;
        }

        public double centerErrorDeg() {
            if (!isValid()) {
                return Double.NaN;
            }

            return -Math.toDegrees(Math.atan2(robotToHubCenter.getY(), robotToHubCenter.getX()));
        }

        public double robotSpaceX() {
            return isValid() ? robotToHubCenter.getX() : Double.NaN;
        }

        public double robotSpaceY() {
            return isValid() ? robotToHubCenter.getY() : Double.NaN;
        }

        public boolean poseBasedEstimate() {
            return isValid();
        }
    }

    private HubTagVision() {}

    public static HubCenterEstimate estimatePreferredFuelHubCenterRobotSpace(
        String limelightName,
        Pose2d robotPoseField
    ) {
        HubCenterEstimate bestEstimate = HubCenterEstimate.EMPTY;

        for (int[] hubTagIds : FieldGeometry.getFuelHubAllTagIdCandidates()) {
            HubCenterEstimate candidateEstimate =
                estimateHubCenterRobotSpace(limelightName, robotPoseField, hubTagIds);
            bestEstimate = selectPreferredEstimate(bestEstimate, candidateEstimate);
        }

        return bestEstimate;
    }

    static HubCenterEstimate estimatePreferredFuelHubCenterRobotSpaceRealtime(
        String limelightName,
        Pose2d robotPoseField
    ) {
        HubCenterEstimate bestEstimate = HubCenterEstimate.EMPTY;

        for (int[] hubTagIds : FieldGeometry.getFuelHubAllTagIdCandidates()) {
            HubCenterEstimate candidateEstimate =
                estimateHubCenterRobotSpaceFromRawFiducials(limelightName, robotPoseField, hubTagIds);
            bestEstimate = selectPreferredEstimate(bestEstimate, candidateEstimate);
        }

        return bestEstimate;
    }

    public static HubCenterEstimate estimateHubCenterRobotSpace(String limelightName, int[] hubTagIds) {
        return estimateHubCenterRobotSpaceFromJson(limelightName, hubTagIds);
    }

    public static HubCenterEstimate estimateHubCenterRobotSpace(
        String limelightName,
        Pose2d robotPoseField,
        int[] hubTagIds
    ) {
        HubCenterEstimate rawEstimate =
            estimateHubCenterRobotSpaceFromRawFiducials(limelightName, robotPoseField, hubTagIds);
        if (rawEstimate.isValid()) {
            return rawEstimate;
        }

        return estimateHubCenterRobotSpaceFromJson(limelightName, hubTagIds);
    }

    private static HubCenterEstimate estimateHubCenterRobotSpaceFromJson(
        String limelightName,
        int[] hubTagIds
    ) {
        if (!LimelightHelpers.getTV(limelightName)) {
            return HubCenterEstimate.EMPTY;
        }

        Translation2d[] centerEstimatesByTag = new Translation2d[hubTagIds.length];
        double[] tagDistancesByTag = new double[hubTagIds.length];
        double[] areasByTag = new double[hubTagIds.length];
        java.util.Arrays.fill(tagDistancesByTag, Double.NaN);
        java.util.Arrays.fill(areasByTag, Double.NEGATIVE_INFINITY);

        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        for (LimelightHelpers.LimelightTarget_Fiducial fiducial : results.targets_Fiducials) {
            int tagId = (int) Math.round(fiducial.fiducialID);
            int hubTagIndex = getTagIndex(hubTagIds, tagId);
            if (hubTagIndex < 0) {
                continue;
            }

            Pose2d tagPoseRobotSpace = fiducial.getTargetPose_RobotSpace2D();
            Translation2d robotToTag = tagPoseRobotSpace.getTranslation();
            if (!isFinite(robotToTag)) {
                continue;
            }

            Translation2d tagToHubCenter = FieldGeometry.getFuelHubCenterOffsetFromTagFrame(tagId)
                .orElse(null);
            if (tagToHubCenter == null) {
                continue;
            }

            Translation2d robotToHubCenter =
                robotToTag.plus(tagToHubCenter.rotateBy(tagPoseRobotSpace.getRotation()));
            if (!isFinite(robotToHubCenter)) {
                continue;
            }

            double tagArea =
                Double.isFinite(fiducial.ta) && fiducial.ta > 0.0
                    ? fiducial.ta
                    : 1.0;

            if (centerEstimatesByTag[hubTagIndex] == null || tagArea > areasByTag[hubTagIndex]) {
                centerEstimatesByTag[hubTagIndex] = robotToHubCenter;
                tagDistancesByTag[hubTagIndex] = robotToTag.getNorm();
                areasByTag[hubTagIndex] = tagArea;
            }
        }

        int visibleHubTagCount = 0;
        double totalWeight = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double totalTagDistance = 0.0;

        for (int i = 0; i < centerEstimatesByTag.length; i++) {
            Translation2d centerEstimate = centerEstimatesByTag[i];
            if (centerEstimate == null) {
                continue;
            }

            double weight = areasByTag[i] > 0.0 ? areasByTag[i] : 1.0;
            totalWeight += weight;
            weightedX += centerEstimate.getX() * weight;
            weightedY += centerEstimate.getY() * weight;
            totalTagDistance += tagDistancesByTag[i];
            visibleHubTagCount++;
        }

        if (visibleHubTagCount == 0 || totalWeight <= 0.0) {
            return HubCenterEstimate.EMPTY;
        }

        Translation2d averagedRobotToHubCenter =
            new Translation2d(weightedX / totalWeight, weightedY / totalWeight);
        if (!isFinite(averagedRobotToHubCenter)) {
            return HubCenterEstimate.EMPTY;
        }

        double weightedSquaredError = 0.0;
        for (int i = 0; i < centerEstimatesByTag.length; i++) {
            Translation2d centerEstimate = centerEstimatesByTag[i];
            if (centerEstimate == null) {
                continue;
            }

            double weight = areasByTag[i] > 0.0 ? areasByTag[i] : 1.0;
            double dx = centerEstimate.getX() - averagedRobotToHubCenter.getX();
            double dy = centerEstimate.getY() - averagedRobotToHubCenter.getY();
            weightedSquaredError += weight * ((dx * dx) + (dy * dy));
        }

        return new HubCenterEstimate(
            averagedRobotToHubCenter,
            totalTagDistance / visibleHubTagCount,
            visibleHubTagCount,
            Math.sqrt(weightedSquaredError / totalWeight)
        );
    }

    private static HubCenterEstimate estimateHubCenterRobotSpaceFromRawFiducials(
        String limelightName,
        Pose2d robotPoseField,
        int[] hubTagIds
    ) {
        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightName);
        if (rawFiducials.length == 0) {
            return HubCenterEstimate.EMPTY;
        }

        Translation2d hubCenterField = FieldGeometry.getFuelHubCenter(hubTagIds);
        Rotation2d fieldToRobot = robotPoseField.getRotation().unaryMinus();

        Translation2d[] centerEstimatesByTag = new Translation2d[hubTagIds.length];
        double[] tagDistancesByTag = new double[hubTagIds.length];
        double[] areasByTag = new double[hubTagIds.length];
        java.util.Arrays.fill(tagDistancesByTag, Double.NaN);
        java.util.Arrays.fill(areasByTag, Double.NEGATIVE_INFINITY);

        for (LimelightHelpers.RawFiducial rawFiducial : rawFiducials) {
            int hubTagIndex = getTagIndex(hubTagIds, rawFiducial.id);
            if (hubTagIndex < 0) {
                continue;
            }

            if (!Double.isFinite(rawFiducial.txnc)) {
                continue;
            }

            double rangeMeters = getRawFiducialRangeMeters(rawFiducial);
            if (!Double.isFinite(rangeMeters) || rangeMeters <= 0.0) {
                continue;
            }

            double planarRangeMeters = getPlanarRangeMeters(rawFiducial, rangeMeters);
            if (!Double.isFinite(planarRangeMeters) || planarRangeMeters <= 0.0) {
                continue;
            }

            Translation2d robotToTag =
                new Translation2d(planarRangeMeters, Rotation2d.fromDegrees(-rawFiducial.txnc));
            if (!isFinite(robotToTag)) {
                continue;
            }

            Translation2d tagFieldTranslation = FieldGeometry.FIELD_LAYOUT.getTagPose(rawFiducial.id)
                .map(tagPose -> tagPose.toPose2d().getTranslation())
                .orElse(null);
            if (tagFieldTranslation == null) {
                continue;
            }

            Translation2d tagToHubRobot =
                hubCenterField.minus(tagFieldTranslation).rotateBy(fieldToRobot);
            Translation2d robotToHubCenter = robotToTag.plus(tagToHubRobot);
            if (!isFinite(robotToHubCenter)) {
                continue;
            }

            double tagArea =
                Double.isFinite(rawFiducial.ta) && rawFiducial.ta > 0.0
                    ? rawFiducial.ta
                    : 1.0;

            if (centerEstimatesByTag[hubTagIndex] == null || tagArea > areasByTag[hubTagIndex]) {
                centerEstimatesByTag[hubTagIndex] = robotToHubCenter;
                tagDistancesByTag[hubTagIndex] = planarRangeMeters;
                areasByTag[hubTagIndex] = tagArea;
            }
        }

        int visibleHubTagCount = 0;
        double totalWeight = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double totalTagDistance = 0.0;

        for (int i = 0; i < centerEstimatesByTag.length; i++) {
            Translation2d centerEstimate = centerEstimatesByTag[i];
            if (centerEstimate == null) {
                continue;
            }

            double weight = areasByTag[i] > 0.0 ? areasByTag[i] : 1.0;
            totalWeight += weight;
            weightedX += centerEstimate.getX() * weight;
            weightedY += centerEstimate.getY() * weight;
            totalTagDistance += tagDistancesByTag[i];
            visibleHubTagCount++;
        }

        if (visibleHubTagCount == 0 || totalWeight <= 0.0) {
            return HubCenterEstimate.EMPTY;
        }

        Translation2d averagedRobotToHubCenter =
            new Translation2d(weightedX / totalWeight, weightedY / totalWeight);
        if (!isFinite(averagedRobotToHubCenter)) {
            return HubCenterEstimate.EMPTY;
        }

        double weightedSquaredError = 0.0;
        for (int i = 0; i < centerEstimatesByTag.length; i++) {
            Translation2d centerEstimate = centerEstimatesByTag[i];
            if (centerEstimate == null) {
                continue;
            }

            double weight = areasByTag[i] > 0.0 ? areasByTag[i] : 1.0;
            double dx = centerEstimate.getX() - averagedRobotToHubCenter.getX();
            double dy = centerEstimate.getY() - averagedRobotToHubCenter.getY();
            weightedSquaredError += weight * ((dx * dx) + (dy * dy));
        }

        return new HubCenterEstimate(
            averagedRobotToHubCenter,
            totalTagDistance / visibleHubTagCount,
            visibleHubTagCount,
            Math.sqrt(weightedSquaredError / totalWeight)
        );
    }

    private static HubCenterEstimate selectPreferredEstimate(
        HubCenterEstimate currentBest,
        HubCenterEstimate candidate
    ) {
        if (!candidate.isValid()) {
            return currentBest;
        }

        if (!currentBest.isValid()) {
            return candidate;
        }

        if (candidate.visibleHubTagCount() != currentBest.visibleHubTagCount()) {
            return candidate.visibleHubTagCount() > currentBest.visibleHubTagCount()
                ? candidate
                : currentBest;
        }

        if (Double.isFinite(candidate.estimateSpreadMeters())
                && Double.isFinite(currentBest.estimateSpreadMeters())
                && Math.abs(candidate.estimateSpreadMeters() - currentBest.estimateSpreadMeters()) > 1e-6) {
            return candidate.estimateSpreadMeters() < currentBest.estimateSpreadMeters()
                ? candidate
                : currentBest;
        }

        if (Double.isFinite(candidate.averageVisibleTagDistanceMeters())
                && Double.isFinite(currentBest.averageVisibleTagDistanceMeters())
                && Math.abs(
                    candidate.averageVisibleTagDistanceMeters()
                        - currentBest.averageVisibleTagDistanceMeters()
                ) > 1e-6) {
            return candidate.averageVisibleTagDistanceMeters() < currentBest.averageVisibleTagDistanceMeters()
                ? candidate
                : currentBest;
        }

        return currentBest;
    }

    private static int getTagIndex(int[] tagIds, int targetTagId) {
        for (int i = 0; i < tagIds.length; i++) {
            if (tagIds[i] == targetTagId) {
                return i;
            }
        }

        return -1;
    }

    private static double getRawFiducialRangeMeters(LimelightHelpers.RawFiducial rawFiducial) {
        if (Double.isFinite(rawFiducial.distToRobot) && rawFiducial.distToRobot > 0.0) {
            return rawFiducial.distToRobot;
        }

        if (Double.isFinite(rawFiducial.distToCamera) && rawFiducial.distToCamera > 0.0) {
            return rawFiducial.distToCamera;
        }

        return Double.NaN;
    }

    private static double getPlanarRangeMeters(
        LimelightHelpers.RawFiducial rawFiducial,
        double rangeMeters
    ) {
        if (!Double.isFinite(rawFiducial.tync)) {
            return rangeMeters;
        }

        double planarRangeMeters = rangeMeters * Math.cos(Math.toRadians(rawFiducial.tync));
        return Double.isFinite(planarRangeMeters) && planarRangeMeters > 0.0
            ? planarRangeMeters
            : rangeMeters;
    }

    private static boolean isFinite(Translation2d translation) {
        return translation != null
            && Double.isFinite(translation.getX())
            && Double.isFinite(translation.getY())
            && translation.getNorm() > 1e-6;
    }
}
