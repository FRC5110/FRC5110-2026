package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceUtil {

    private static DriverStation.Alliance lastKnownAlliance = null;

    private AllianceUtil() {}

    public static void refresh() {
        DriverStation.getAlliance().ifPresent(alliance -> lastKnownAlliance = alliance);
    }

    public static Optional<DriverStation.Alliance> getLastKnownAlliance() {
        return Optional.ofNullable(lastKnownAlliance);
    }

    public static DriverStation.Alliance getAllianceOrBlue() {
        return getLastKnownAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public static boolean isRed() {
        return getAllianceOrBlue() == DriverStation.Alliance.Red;
    }

    public static boolean isBlue() {
        return !isRed();
    }

    public static String getAllianceLabel() {
        return getLastKnownAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red ? "Red" : "Blue")
            .orElse("Unknown->BlueFallback");
    }
}
