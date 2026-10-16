package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BlinkinLedSubsystem extends SubsystemBase {

    public enum LedState {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST,
        HUB_TAG_FOUND,
        HUB_TAG_MISSING,
        CLIMB_SEARCHING,
        CLIMB_ALIGNING,
        CLIMB_ALIGNED,
        ERROR
    }

    // REV Blinkin pattern values
    private static final double PATTERN_DISABLED = 0.87;   // Solid Blue
    private static final double PATTERN_AUTONOMOUS = -0.07; // Gold strobe
    private static final double PATTERN_TELEOP = 0.65;      // Solid Orange
    private static final double PATTERN_TEST = 0.57;        // Solid Hot Pink
    private static final double PATTERN_HUB_TAG_FOUND = 0.75;   // Solid Dark Green
    private static final double PATTERN_HUB_TAG_MISSING = 0.61; // Solid Red
    private static final double PATTERN_CLIMB_SEARCHING = -0.07; // Gold strobe
    private static final double PATTERN_CLIMB_ALIGNING = -0.11;  // Breath Red
    private static final double PATTERN_CLIMB_ALIGNED = 0.75;    // Solid Dark Green
    private static final double PATTERN_ERROR = 0.61;       // Solid Red

    private final Spark blinkin = new Spark(Constants.PwmPorts.BLINKIN);
    private LedState currentState = LedState.DISABLED;

    public BlinkinLedSubsystem() {
        setState(LedState.DISABLED);
    }

    public void setState(LedState state) {
        currentState = state;
        blinkin.set(patternFor(state));
    }

    public LedState getState() {
        return currentState;
    }

    private double patternFor(LedState state) {
        return switch (state) {
            case DISABLED -> PATTERN_DISABLED;
            case AUTONOMOUS -> PATTERN_AUTONOMOUS;
            case TELEOP -> PATTERN_TELEOP;
            case TEST -> PATTERN_TEST;
            case HUB_TAG_FOUND -> PATTERN_HUB_TAG_FOUND;
            case HUB_TAG_MISSING -> PATTERN_HUB_TAG_MISSING;
            case CLIMB_SEARCHING -> PATTERN_CLIMB_SEARCHING;
            case CLIMB_ALIGNING -> PATTERN_CLIMB_ALIGNING;
            case CLIMB_ALIGNED -> PATTERN_CLIMB_ALIGNED;
            case ERROR -> PATTERN_ERROR;
        };
    }

    @Override
    public void periodic() {
        // Realtime dashboard publishing disabled to reduce loop overflows.
        // SmartDashboard.putString("LED/State", currentState.name());
    }
}
