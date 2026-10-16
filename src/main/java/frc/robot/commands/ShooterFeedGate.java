package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShooterFeedGate {

    private enum FeedState {
        WAIT_READY,
        COMMIT_PACKET,
        RECOVER
    }

    private FeedState feedState = FeedState.WAIT_READY;
    private double packetStartSec = Double.NaN;
    private double recoveryStartSec = Double.NaN;

    public void reset() {
        feedState = FeedState.WAIT_READY;
        packetStartSec = Double.NaN;
        recoveryStartSec = Double.NaN;
    }

    // Commit a short feed packet once the shooter is stably ready, then force
    // a brief recovery pause before the next packet can start.
    public boolean shouldFeed(ShooterFlywheelSubsystem shooter) {
        double nowSec = Timer.getFPGATimestamp();
        double packetSec = Math.max(0.0, Constants.ShooterTune.FEED_PACKET_SEC_DEFAULT);
        double recoveryMinSec = Math.max(0.0, Constants.ShooterTune.FEED_RECOVERY_MIN_SEC_DEFAULT);

        switch (feedState) {
            case WAIT_READY:
                if (shooter.isReadyToFeed()) {
                    feedState = FeedState.COMMIT_PACKET;
                    packetStartSec = nowSec;
                    recoveryStartSec = Double.NaN;
                    return true;
                }
                return false;

            case COMMIT_PACKET:
                if (!Double.isFinite(packetStartSec)) {
                    packetStartSec = nowSec;
                }

                if ((nowSec - packetStartSec) >= packetSec) {
                    feedState = FeedState.RECOVER;
                    recoveryStartSec = nowSec;
                    return false;
                }
                return true;

            case RECOVER:
                if (!Double.isFinite(recoveryStartSec)) {
                    recoveryStartSec = nowSec;
                }

                if ((nowSec - recoveryStartSec) < recoveryMinSec) {
                    return false;
                }

                if (shooter.isReadyToFeed()) {
                    feedState = FeedState.COMMIT_PACKET;
                    packetStartSec = nowSec;
                    recoveryStartSec = Double.NaN;
                    return true;
                }
                return false;

            default:
                reset();
                return false;
        }
    }

    public boolean isLatched() {
        return feedState == FeedState.COMMIT_PACKET;
    }
}
