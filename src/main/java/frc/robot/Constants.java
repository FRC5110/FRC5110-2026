package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {

    private Constants() {}

    public static final class CanBus {
        public static final String RIO = "rio";

        private CanBus() {}
    }

    public static final class CanIds {
        public static final int INTAKE_ROLLER = 14;
        //retire 15
        public static final int CONVEYOR_ARM_LEADER = 20;
        public static final int CONVEYOR_ARM_FOLLOWER = 21;
        public static final int CONVEYOR = 16;
        public static final int SHOOTER_FEED_WHEEL = 19;
        public static final int SHOOTER_FLYWHEEL_LEADER = 32;
        public static final int SHOOTER_FLYWHEEL_FOLLOWER = 33;
        // Legacy aliases retained so older code paths still compile.
        public static final int SHOOTER_FLYWHEEL_LEADER_KRAKEN = SHOOTER_FLYWHEEL_LEADER;
        public static final int SHOOTER_FLYWHEEL_FOLLOWER_KRAKEN = SHOOTER_FLYWHEEL_FOLLOWER;
        //public static final int CLIMBER_MOTOR = 18;

        public static final int PIGEON2 = 13;

        public static final int FRONT_LEFT_DRIVE = 5;
        public static final int FRONT_LEFT_STEER = 4;
        public static final int FRONT_LEFT_ENCODER = 6;

        public static final int FRONT_RIGHT_DRIVE = 8;
        public static final int FRONT_RIGHT_STEER = 7;
        public static final int FRONT_RIGHT_ENCODER = 9;

        public static final int BACK_LEFT_DRIVE = 2;
        public static final int BACK_LEFT_STEER = 1;
        public static final int BACK_LEFT_ENCODER = 3;

        public static final int BACK_RIGHT_DRIVE = 11;
        public static final int BACK_RIGHT_STEER = 10;
        public static final int BACK_RIGHT_ENCODER = 12;

        private CanIds() {}
    }

    public static final class PwmPorts {
        public static final int BLINKIN = 1;
        public static final int CLIMBER_LATCH_SERVO = 0;

        private PwmPorts() {}
    }

    
    public static final class Vision {
        public static final class Front {
            public static final String LIMELIGHT_NAME = "limelight";
            // Match this to the AprilTag pipeline index configured on the Limelight.
            public static final int APRILTAG_PIPELINE_INDEX = 0;

            private Front() {}
        }

        private Vision() {}
    }

    public static final class Logging {
        public static final String ENABLE_CTRE_SIGNAL_LOGGER_KEY = "Logging/EnableCTRESignalLogger";
        public static final String CTRE_SIGNAL_LOG_EVERY_N_LOOPS_KEY = "Logging/CtreSignalLogEveryNLoops";

        public static final boolean ENABLE_CTRE_SIGNAL_LOGGER_DEFAULT = false;
        public static final int CTRE_SIGNAL_LOG_EVERY_N_LOOPS_DEFAULT = 5;
        // Keep WPILib datalogging off until the roboRIO startup issue is resolved.
        public static final boolean WPILIB_DATA_LOGGING_ENABLED = false;
        // Use an explicit path when re-enabling so startup does not depend on USB log auto-detect.
        public static final String WPILIB_DATA_LOGGING_DIR = "/home/lvuser/logs";

        private Logging() {}
    }

    public static final class Calibration {
        // Turn these on only when actively testing or calibrating.
        public static final boolean ALIGN_TO_HUB_CALIBRATE = false;
        public static final boolean SHOOTER_DISTANCE_CALIBRATE = false;
        public static final boolean HUB_DISTANCE_CALIBRATE = false;
        public static final boolean SHOOTERFLYWHEEL_CONVEYOR_TUNE = false;
        public static final boolean INTAKE_ARM_CALIBRATION = false;
        public static final boolean DRIVE_DEBUG = false;

        private Calibration() {}
    }

    public static final class ShooterTune {
        public static final String AT_SPEED_TOL_RPM_KEY = "Shooter/Tune/AtSpeedTolRPM";
        public static final String AT_SPEED_STABLE_SEC_KEY = "Shooter/Tune/AtSpeedStableSec";
        public static final String BOOST_ENABLED_KEY = "Shooter/Tune/BoostEnabled";
        public static final String BOOST_VOLTS_REQUESTED_KEY = "Shooter/Tune/BoostVoltsRequested";
        public static final String BOOST_ENTER_ERROR_RPM_KEY = "Shooter/Tune/BoostEnterErrorRPM";
        public static final String BOOST_EXIT_ERROR_RPM_KEY = "Shooter/Tune/BoostExitErrorRPM";
        public static final String HANDOFF_TIME_SEC_KEY = "Shooter/Tune/HandoffTimeSec";
        public static final String MIN_SLOPE_RPM_PER_SEC_KEY = "Shooter/Tune/MinSlopeRPMperSec";
        public static final String V_BAT_HIGH_KEY = "Shooter/Tune/VbatHigh";
        public static final String V_BAT_MID_KEY = "Shooter/Tune/VbatMid";
        public static final String V_BAT_LOW_KEY = "Shooter/Tune/VbatLow";
        public static final String BOOST_MAX_AT_HIGH_V_KEY = "Shooter/Tune/BoostMaxAtHighV";
        public static final String BOOST_MAX_AT_MID_V_KEY = "Shooter/Tune/BoostMaxAtMidV";
        public static final String BOOST_MAX_AT_LOW_V_KEY = "Shooter/Tune/BoostMaxAtLowV";
        public static final String BOOST_MAX_AT_VERY_LOW_V_KEY = "Shooter/Tune/BoostMaxAtVeryLowV";
        public static final String OVERRIDE_ENABLED_KEY = "Shooter/Tune/OverrideEnabled";
        public static final String OVERRIDE_RPM_KEY = "Shooter/Tune/OverrideRPM";
        public static final String PRESET_SHOT_RPM_KEY = "Shooter/Tune/PresetShotRPM";
        public static final String USE_DISTANCE_RPM_KEY = "Shooter/Tune/UseDistanceRPM";
        public static final String LIMELIGHT_HUB_DISTANCE_ALPHA_KEY = "Shooter/Tune/LimelightHubDistanceAlpha";
        public static final String LIMELIGHT_HUB_MIN_TAGS_KEY = "Shooter/Tune/LimelightHubMinTags";
        public static final String LIMELIGHT_HUB_MAX_AVG_TAG_DIST_KEY = "Shooter/Tune/LimelightHubMaxAvgTagDistM";
        public static final String HUB_SINGLE_TAG_CENTER_OFFSET_METERS_KEY =
            "Shooter/Tune/HubSingleTagCenterOffsetMeters";
        public static final String PAUSE_FEED_WHEN_BOOST_KEY = "Shooter/Tune/PauseFeedWhenBoost";
        public static final String CONVEYOR_FEED_POWER_KEY = "Shooter/Tune/ConveyorFeedPower";
        public static final String CONVEYOR_HOLD_POWER_KEY = "Shooter/Tune/ConveyorHoldPower";
        public static final String FEED_READY_STABLE_SEC_KEY = "Shooter/Tune/FeedReadyStableSec";
        public static final String FEED_PACKET_SEC_KEY = "Shooter/Tune/FeedPacketSec";
        public static final String FEED_RECOVERY_MIN_SEC_KEY = "Shooter/Tune/FeedRecoveryMinSec";
        public static final String FEED_RPM_FILTER_ALPHA_KEY = "Shooter/Tune/FeedRpmFilterAlpha";
        public static final String PID_P_KEY = "Shooter/Tune/PID/P";
        public static final String PID_I_KEY = "Shooter/Tune/PID/I";
        public static final String PID_D_KEY = "Shooter/Tune/PID/D";
        public static final String PID_FF_KEY = "Shooter/Tune/PID/VelocityFF";
        public static final String CURRENT_LIMIT_AMPS_KEY = "Shooter/Tune/CurrentLimitAmps";
        public static final String MIN_RPM_KEY = "Shooter/Tune/MinRPM";
        public static final String MAX_RPM_KEY = "Shooter/Tune/MaxRPM";
        public static final String DISTANCE_POINT_METERS_KEY_PREFIX = "Shooter/Tune/DistancePointMeters/";
        public static final String RPM_POINT_KEY_PREFIX = "Shooter/Tune/RpmPoint/";
        public static final String DISTANCE_RPM_TRIM_KEY = "Shooter/Tune/DistanceRPMTrim";

        public static final double AT_SPEED_TOL_RPM_DEFAULT = 125.0;
        public static final double AT_SPEED_STABLE_SEC_DEFAULT = 0.03;
        public static final boolean BOOST_ENABLED_DEFAULT = false;
        public static final double BOOST_VOLTS_REQUESTED_DEFAULT = 8.5;
        public static final double BOOST_ENTER_ERROR_RPM_DEFAULT = 225.0;
        public static final double BOOST_EXIT_ERROR_RPM_DEFAULT = 90.0;
        public static final double HANDOFF_TIME_SEC_DEFAULT = 0.08;
        public static final double MIN_SLOPE_RPM_PER_SEC_DEFAULT = 250.0;
        public static final double V_BAT_HIGH_DEFAULT = 12.0;
        public static final double V_BAT_MID_DEFAULT = 11.2;
        public static final double V_BAT_LOW_DEFAULT = 10.5;
        public static final double BOOST_MAX_AT_HIGH_V_DEFAULT = 11.5;
        public static final double BOOST_MAX_AT_MID_V_DEFAULT = 10.5;
        public static final double BOOST_MAX_AT_LOW_V_DEFAULT = 9.0;
        public static final double BOOST_MAX_AT_VERY_LOW_V_DEFAULT = 7.0;
        public static final boolean OVERRIDE_ENABLED_DEFAULT = false;
        public static final double OVERRIDE_RPM_DEFAULT = RobotContainer.TEST_SHOOT_RPM;
        public static final double PRESET_SHOT_RPM_DEFAULT = RobotContainer.TEST_SHOOT_RPM;
        public static final boolean USE_DISTANCE_RPM_DEFAULT = false;
        public static final double LIMELIGHT_HUB_DISTANCE_ALPHA_DEFAULT = 0.35;
        public static final int LIMELIGHT_HUB_MIN_TAGS_DEFAULT = 1;
        public static final double LIMELIGHT_HUB_MAX_AVG_TAG_DIST_DEFAULT = 7.0;
        public static final double HUB_SINGLE_TAG_CENTER_OFFSET_METERS_DEFAULT = 0.1524;
        public static final double CONVEYOR_HOLD_POWER_DEFAULT = 0.0;
   
        //**************************************************************************
        //Tuners for Shoot/Conveyor coordination
        //************************************************************************** 
        public static final double FEED_READY_TOL_RPM_DEFAULT = 175.0;
        public static final double FEED_READY_STABLE_SEC_DEFAULT = 0.08;
        public static final double FEED_RPM_FILTER_ALPHA_DEFAULT = 0.30;
        public static final double CONVEYOR_FEED_POWER_DEFAULT = RobotContainer.SHOOT_CONVEYOR_PERCENT;
        public static final double FEED_PACKET_SEC_DEFAULT = 0.22;
        public static final double FEED_RECOVERY_MIN_SEC_DEFAULT = 0.06;
        public static final boolean PAUSE_FEED_WHEN_BOOST_DEFAULT = true;
        
        public static final double PID_P_DEFAULT = 0.004;//changed from 6
        public static final double PID_I_DEFAULT = 0.0;
        public static final double PID_D_DEFAULT = 0.0;
        // SysId-style feedforward constants are stored in rotations/sec units.
        // Current shooter compatibility mode actively applies only kV through Spark velocityFF.
        // kS and kA are retained for documentation and future software feedforward work.
        public static final double FEEDFORWARD_KS_VOLTS_DEFAULT = 0.086036;
        public static final double FEEDFORWARD_KV_VOLTS_PER_RPS_DEFAULT = 0.11221;
        public static final double FEEDFORWARD_KA_VOLTS_PER_RPS2_DEFAULT = 0.034568;

        public static final boolean FLYWHEEL_LEADER_INVERTED_DEFAULT = true;
        public static final boolean FLYWHEEL_FOLLOWER_OPPOSES_LEADER_DEFAULT = true;
        public static final boolean FLYWHEEL_BRAKE_MODE_DEFAULT = false;
        public static final double FLYWHEEL_MOTOR_TO_WHEEL_RATIO_DEFAULT = 1.0;
        public static final double FLYWHEEL_SLOT0_KP_DEFAULT = 2.0;
        public static final double FLYWHEEL_SLOT0_KI_DEFAULT = 0.0;
        public static final double FLYWHEEL_SLOT0_KD_DEFAULT = 0.0;
        public static final double FLYWHEEL_SLOT0_KS_DEFAULT = 0.0;
        public static final double FLYWHEEL_SLOT0_KV_DEFAULT = 0.0;
        public static final double FLYWHEEL_SLOT0_KA_DEFAULT = 0.0;
        public static final double FLYWHEEL_STATOR_CURRENT_LIMIT_AMPS_DEFAULT = 120.0;
        public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT_AMPS_DEFAULT = 60.0;
        public static final double FLYWHEEL_SUPPLY_CURRENT_LOWER_LIMIT_AMPS_DEFAULT = 40.0;
        public static final double FLYWHEEL_SUPPLY_CURRENT_LOWER_TIME_SEC_DEFAULT = 0.5;
        public static final double FLYWHEEL_PEAK_FORWARD_TORQUE_CURRENT_AMPS_DEFAULT = 120.0;
        public static final double FLYWHEEL_PEAK_REVERSE_TORQUE_CURRENT_AMPS_DEFAULT = -120.0;
        public static final double FLYWHEEL_CONTROL_UPDATE_HZ_DEFAULT = 100.0;
        public static final double FLYWHEEL_STATUS_UPDATE_HZ_DEFAULT = 100.0;
        public static final double FEED_WHEEL_WHEN_SHOOTING_RPM_DEFAULT = 6000.0;
        public static final boolean FEED_WHEEL_INVERTED_DEFAULT = true;
        public static final boolean FEED_WHEEL_BRAKE_MODE_DEFAULT = false;
        public static final double FEED_WHEEL_MOTOR_TO_WHEEL_RATIO_DEFAULT = 1.0;
        public static final double FEED_WHEEL_PID_P_DEFAULT = 0.0005;
        public static final double FEED_WHEEL_PID_I_DEFAULT = 0.0;
        public static final double FEED_WHEEL_PID_D_DEFAULT = 0.0;
        public static final double FEED_WHEEL_PID_FF_DEFAULT = 0.00175;
        public static final int FEED_WHEEL_CURRENT_LIMIT_AMPS_DEFAULT = 50;
        public static final double FEED_WHEEL_MAX_RPM_DEFAULT = 6500.0;
        public static final double FEED_WHEEL_HEALTH_MIN_COMMAND_RPM_DEFAULT = 1000.0;
        public static final double FEED_WHEEL_HEALTH_MIN_MEASURED_RPM_DEFAULT = 150.0;
        public static final double FEED_WHEEL_HEALTH_FAULT_DELAY_SEC_DEFAULT = 0.25;
        public static final double TELEMETRY_PUBLISH_SEC_DEFAULT = 0.1;



        public static final double SYSID_RAMP_RATE_VOLTS_PER_SEC_DEFAULT = 1.0;
        public static final double SYSID_STEP_VOLTAGE_DEFAULT = 7.0;
        public static final double SYSID_TIMEOUT_SEC_DEFAULT = 10.0;
        public static final int CURRENT_LIMIT_AMPS_DEFAULT = 60;
        public static final double MIN_RPM_DEFAULT = -6200.0;
        public static final double MAX_RPM_DEFAULT = 6200.0;
        public static final boolean FOLLOWER_INVERTED_DEFAULT = false;

        

        public static final double FEED_DROPOUT_TOL_RPM_DEFAULT = 325.0;
        //Adjust the shooting RPM speed (+/-) based on what we are seeing on the field
        public static final double DISTANCE_RPM_TRIM_DEFAULT = 0.0;

        public static final double[] DISTANCE_POINT_METERS_DEFAULTS = {
            1.5, 2.3, 2.65, 2.85, 3.1, 3.4, 3.7 
        };
        public static final double[] RPM_POINT_DEFAULTS = {
            1650, 2050, 2175, 2200, 2400, 2600, 3100
        };
        public static final int SHOOT_THREE_FIRST_FUEL_COUNT_DEFAULT = 3;
        public static final double SHOOT_THREE_CONVEYOR_ROTATIONS_PER_FUEL_DEFAULT = 1.0;
        public static final double SHOOT_THREE_ARM_DOWN_SETPOINT_ROT_DEFAULT = -13300.0 / 360.0;
        public static final double SHOOT_THREE_ARM_TOLERANCE_ROT_DEFAULT = 0.02;

        public static String distancePointMetersKey(int index) {
            return DISTANCE_POINT_METERS_KEY_PREFIX + (index + 1);
        }

        public static String rpmPointKey(int index) {
            return RPM_POINT_KEY_PREFIX + (index + 1);
        }

        private ShooterTune() {}
    }

    public static final class ConveyorArm {
        public static final String CAN_BUS = CanBus.RIO;
        public static final int ABSOLUTE_ENCODER_DIO_PORT = 0;
        public static final boolean LEADER_INVERTED = false;
        public static final boolean FOLLOWER_OPPOSES_LEADER = true;
        public static final boolean BRAKE_MODE = true;
        public static final double MANUAL_SPEED_SCALE_DEFAULT = 0.45;
        public static final double MAX_MANUAL_OUTPUT = 0.5;
        public static final double ABSOLUTE_ENCODER_OFFSET_ROTATIONS = 0.0;
        public static final boolean ABSOLUTE_ENCODER_INVERTED = false;
        // Conveyor-arm mechanical limits in absolute-encoder degrees.
        // Replace these full-range placeholders with measured values before using presets.
        // The configured range must not cross the encoder's 0/360-degree wrap point.
        public static final double ABSOLUTE_ENCODER_LOW_LIMIT_DEGREES = 0.0;
        public static final double ABSOLUTE_ENCODER_HIGH_LIMIT_DEGREES = 360.0;
        // Verify this at low manual speed before configuring the endpoint values.
        public static final boolean POSITIVE_OUTPUT_INCREASES_ABSOLUTE_ENCODER_DEGREES = true;
        public static final double PRESET_POSITION_TOLERANCE_DEGREES = 2.0;
        public static final double PRESET_POSITION_KP_PER_DEGREE = 0.012;
        public static final double PRESET_MIN_OUTPUT = 0.08;
        // DIO absolute-encoder position hold. POSITION_KP is volts per degree of error.
        // Tune HOLD_KG_VOLTS on the robot after setting ENCODER_DEGREES_AT_HORIZONTAL.
        public static final double POSITION_KP_VOLTS_PER_DEGREE = 0.144;
        public static final double POSITION_KI_VOLTS_PER_DEGREE_SECOND = 0.0;
        public static final double POSITION_KD_VOLTS_PER_DEGREE_PER_SECOND = 0.0;
        public static final double HOLD_KG_VOLTS = 0.0;
        public static final double ENCODER_DEGREES_AT_HORIZONTAL = 0.0;
        public static final double MAX_CLOSED_LOOP_VOLTS = MAX_MANUAL_OUTPUT * 12.0;
        public static final double PRESET_TIMEOUT_SEC = 5.0;
        public static final double PRESET_WRONG_DIRECTION_TOLERANCE_DEGREES = 2.0;
        public static final double PRESET_DIRECTION_CHECK_DELAY_SEC = 0.15;
        public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 40.0;
        public static final double SUPPLY_CURRENT_LOWER_TIME_SEC = 0.5;
        public static final double CONTROL_UPDATE_HZ = 100.0;
        public static final double STATUS_UPDATE_HZ = 50.0;

        private ConveyorArm() {}
    }

    public static final class RobotContainer {
        public static final double MAX_SPEED = 4.5;
        public static final double MAX_ROT = 6.0;
        public static final double DRIVER_TRANSLATION_INVERT = -1.0;
        public static final double DRIVER_ROTATION_INVERT = -1.0;
        // DEBUG_TELEOP_FRONT: Mechanical/vision front is the shooter side,
        // but the driver wants teleop heading commands to feel intake-front.
        public static final double DRIVER_TELEOP_FRONT_OFFSET_DEG = 0; //180.0; changed to 0 on 3/24
        public static final double HEADING_HOLD_TOLERANCE_DEG = 1.5;

        public static final double DEADBAND = 0.10;
        public static final double CLIMBER_MAX_OUTPUT = 0.6;
        public static final double TRIGGER_DEADBAND = 0.05;
        public static final double TEST_SHOOT_RPM = 3520.0; //max rpm is 6784
        public static final double SHOOT_CONVEYOR_PERCENT = 0.30;
        public static final double FUEL_HUB_ALIGN_TIMEOUT_SEC = 1.5;
        public static final double FUEL_HUB_ALIGN_ACQUIRE_TIMEOUT_SEC = 0.4;
        public static final double FUEL_HUB_ALIGN_LOST_TARGET_TIMEOUT_SEC = 0.2;
        public static final double FUEL_HUB_ALIGN_HEADING_LATCH_SEC = 0.25;
        public static final double FIELD_CENTRIC_ALIGN_TIMEOUT_SEC = 2.5;

        public static final Pose2d BLUE_OUTPOST =
            new Pose2d(4.5, 3.0, new Rotation2d());
        public static final Pose2d RED_OUTPOST =
            new Pose2d(12.0, 3.0, Rotation2d.fromDegrees(180));

        private RobotContainer() {}
    }
}
