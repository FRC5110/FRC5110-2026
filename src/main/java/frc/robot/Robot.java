package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.BlinkinLedSubsystem.LedState;

public class Robot extends TimedRobot {
    private static final int ELASTIC_LAYOUT_SERVER_PORT = 5810;

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();
        startWpilibDataLogging();
        configureLimelightPortForwards();
        WebServer.start(ELASTIC_LAYOUT_SERVER_PORT, Filesystem.getDeployDirectory().getAbsolutePath());
        AllianceUtil.refresh();

        m_robotContainer = new RobotContainer();

        // Nothing extra here; RobotContainer already schedules vision updates
        m_robotContainer.setLedState(LedState.DISABLED);
        m_robotContainer.setVisionOdometryEnabled(false);
    }

    @Override
    public void robotPeriodic() {
        AllianceUtil.refresh();
        // Runs the scheduler, including default commands
        CommandScheduler.getInstance().run();
        m_robotContainer.updateTelemetry();
    }

    @Override
    public void autonomousInit() {
        boolean allianceKnownBeforeRefresh =
            DriverStation.getAlliance().isPresent() || AllianceUtil.getLastKnownAlliance().isPresent();
        AllianceUtil.refresh();
        SmartDashboard.putString("Auto/AllianceAtStart", AllianceUtil.getAllianceLabel());
        SmartDashboard.putBoolean("Auto/ShouldFlipForRed", AllianceUtil.isRed());
        if (!allianceKnownBeforeRefresh && DriverStation.getAlliance().isEmpty()) {
            DriverStation.reportWarning(
                "Autonomous started with unknown alliance; using blue fallback until alliance is known.",
                false
            );
        }

        m_robotContainer.warmHubDistanceVisionIfNeeded();
        m_robotContainer.setLedState(LedState.AUTONOMOUS);
        m_robotContainer.forceMatchTelemetryProfile();
        // Temporarily keep Limelight vision odometry disabled in autonomous.
        // m_robotContainer.setVisionOdometryEnabled(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        m_robotContainer.setLedState(LedState.TELEOP);
        m_robotContainer.forceMatchTelemetryProfile();
        m_robotContainer.setVisionOdometryEnabled(false);
        // Cancel autonomous when teleop starts
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledInit() {
        // Optionally reset anything needed when disabled
        m_robotContainer.setLedState(LedState.DISABLED);
        m_robotContainer.setVisionOdometryEnabled(false);
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.warmHubDistanceVisionIfNeeded();
    }

    @Override
    public void testInit() {
        m_robotContainer.setLedState(LedState.TEST);
        m_robotContainer.setVisionOdometryEnabled(false);
        // Cancel all running commands during test
        CommandScheduler.getInstance().cancelAll();
    }

    private void startWpilibDataLogging() {
        if (!Constants.Logging.WPILIB_DATA_LOGGING_ENABLED) {
            DriverStation.reportWarning(
                "WPILib datalogging disabled in code to avoid startup bootloop.",
                false
            );
            return;
        }

        String logDir = Constants.Logging.WPILIB_DATA_LOGGING_DIR;
        if (logDir == null || logDir.isBlank()) {
            DataLogManager.start();
        } else {
            DataLogManager.start(logDir);
        }
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    private void configureLimelightPortForwards() {
        String limelightHost = Constants.Vision.Front.LIMELIGHT_NAME + ".local";
        addPortForwardRange(5800, limelightHost, 5800);

        DriverStation.reportWarning(
            "Limelight port forwards active: rio:5801 -> " + limelightHost + ":5801",
            false
        );
    }

    private void addPortForwardRange(int localBasePort, String host, int remoteBasePort) {
        for (int i = 0; i < 10; i++) {
            PortForwarder.add(localBasePort + i, host, remoteBasePort + i);
        }
    }
}
