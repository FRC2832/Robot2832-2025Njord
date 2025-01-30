// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static final double kDefaultPeriod = 0.02;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @SuppressWarnings("unused")
  private Logger logger;

  ArrayList<Runnable> periodics;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    SmartDashboard.putData(CommandScheduler.getInstance());

    /* This is a fix from 2023 with a Rio2.  We took a very hard hit and the Rio went into brownout
     * protection.  That was fine.  But we stayed stuck in brownout protection throughout power cycles
     * and code deploys.  We never figured out how to clear brownout before the match and missed a
     * playoff match because of it.  The best guess we have was to take out and put back in the SD card,
     * or put a different SD card in the robot, as this cleared some internal FPGA flag.
     *
     * This specific change just lowers the threshold the Rio2 goes into brownout to 3V.  Because the
     * Rio2 needs 5.5V to run, the hardware will shutoff before brownout protection happens, and we
     * should never see Brownout Protection again.
     */
    RobotController.setBrownoutVoltage(3);

    // turn off hot reload at the competition.
    if (DriverStation.isFMSAttached()) {
      PPLibTelemetry.enableCompetitionMode();
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    periodics = new ArrayList<>();
    m_robotContainer = new RobotContainer(this);
    m_robotContainer.configureBindings();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    for (Runnable runnable : periodics) {
      runnable.run();
    }
    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }

    // this clears all the old polled triggers
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // since we sometimes switch configurations based on controller type,
    // reset the bindings
    m_robotContainer.configureBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
   * Adds a periodic function to our code. AdvantageKit's Logged Robot does not supply this, so we
   * made our own. Note: The period and offset are ignored for now.
   *
   * @param callback Function to run
   * @param periodSeconds Ignored, set to our looptime (20ms)
   * @param offsetSeconds Ignored
   * @return
   */
  public boolean addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds) {
    periodics.add(callback);
    return true;
  }
}
