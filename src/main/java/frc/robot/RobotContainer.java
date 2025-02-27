// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.clawintake.ClawIntake;
import frc.robot.clawintake.ClawIntakeHw;
import frc.robot.clawintake.ClawIntakeSim;
import frc.robot.clawpivot.ClawPivot;
import frc.robot.clawpivot.ClawPivotHw;
import frc.robot.clawpivot.ClawPivotSim;
import frc.robot.clawpivot.PlaySong;
import frc.robot.controllers.DriverControls;
import frc.robot.controllers.OperatorControls;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorHw;
import frc.robot.elevator.ElevatorSimul;
import frc.robot.piecetypeswitcher.PieceTypeSwitcher;
import frc.robot.piecetypeswitcher.ScoringPositions;
import frc.robot.simulation.RobotSim;
import frc.robot.swervedrive.SwerveSubsystem;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.Vision;
import java.io.File;
import java.util.Set;
import org.livoniawarriors.LoopTimeLogger;
import org.livoniawarriors.PdpLoggerKit;
import org.livoniawarriors.leds.BreathLeds;
import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.motorcontrol.MotorControls;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private SwerveSubsystem swerveDrive;

  // private FrontLeds frontLeds;
  // private RearLeds rearLeds;
  private LedSubsystem leds;
  private Vision vision;
  private Elevator elevator;
  private ClawPivot pivot;
  private ClawIntake intake;
  private PieceTypeSwitcher pieceTypeSwitcher;

  private SendableChooser<Command> autoChooser;
  private AprilTagCamera frontCamera;

  public final String CTRE_CAN_BUS = "Njord";
  private final String[] PDP_CHANNEL_NAMES = {
    "Channel 0",
    "Channel 1",
    "Channel 2",
    "Channel 3",
    "Channel 4",
    "Channel 5",
    "Channel 6",
    "Channel 7",
    "Channel 8",
    "Channel 9",
    "Channel 10",
    "Channel 11",
    "Channel 12",
    "Channel 13",
    "Channel 14",
    "Channel 15",
    "Channel 16",
    "Channel 17",
    "Channel 18",
    "Channel 19",
    "Channel 20",
    "Channel 21",
    "Channel 22",
    "Channel 23"
  };

  public RobotContainer(Robot robot) {
    String swerveDirectory = "swerve/njord";
    // subsystems used in all robots
    swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveDirectory));
    // frontLeds = new FrontLeds(0, 54);
    // rearLeds = new RearLeds(frontLeds);
    leds = new LedSubsystem(0, 320);
    pieceTypeSwitcher = new PieceTypeSwitcher();

    if (Robot.isSimulation()) {
      // drive fast in simulation
      swerveDrive.setMaximumSpeed(5, Math.PI);
      // start in red zone since simulation defaults to red 1 station to make field oriented easier
      swerveDrive.resetOdometry(new Pose2d(16.28, 4.03, Rotation2d.fromDegrees(180)));
      intake = new ClawIntakeSim();
      elevator = new ElevatorSimul();
      pivot = new ClawPivotSim();
    } else {
      swerveDrive.setMaximumSpeed(3, Math.toRadians(220));
      intake = new ClawIntakeHw();
      elevator = new ElevatorHw();
      pivot = new ClawPivotHw();
    }

    vision = new Vision(swerveDrive);
    frontCamera =
        new AprilTagCamera(
            "front",
            new Rotation3d(0, Units.degreesToRadians(0), Math.toRadians(0)),
            new Translation3d(0.363, 0, 0.31),
            VecBuilder.fill(4, 4, 8),
            VecBuilder.fill(0.5, 0.5, 1));

    // vision.addCamera(frontCamera);
    // add some buttons to press for development
    SmartDashboard.putData(
        "Fine Drive to Pose",
        swerveDrive.finePosition(new Pose2d(2.75, 4.15, Rotation2d.fromDegrees(0))));

    // Register Named Commands for PathPlanner
    NamedCommands.registerCommand("ScorePieceL1", new WaitCommand(1));
    NamedCommands.registerCommand("GetFromHP", new WaitCommand(2));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData(
        "Clear Sticky Faults", new InstantCommand(MotorControls::ClearStickyFaults));

    SmartDashboard.putData("Play Song", new PlaySong(pivot));

    // periodic tasks to add
    robot.addPeriodic(MotorControls::UpdateLogs, Robot.kDefaultPeriod, 0);
    robot.addPeriodic(new PdpLoggerKit(PDP_CHANNEL_NAMES), Robot.kDefaultPeriod, 0);
    robot.addPeriodic(new DriverFeedback(), Robot.kDefaultPeriod, 0);
    robot.addPeriodic(new RobotSim(swerveDrive::getPose), Robot.kDefaultPeriod, 0);

    new LoopTimeLogger(robot, NetworkTableInstance.getDefault().getTable("Task Timings (ms)"));
    CanBridge.runTCP();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {
    DriverControls driver = new DriverControls();
    OperatorControls op = new OperatorControls();

    Command driveFieldOrientedAnglularVelocity =
        swerveDrive.driveCommand(driver::getDriveX, driver::getDriveY, driver::getTurn);

    driver.getSwerveLockTrigger().whileTrue(swerveDrive.swerveLock());
    driver.isFieldOrientedResetRequestedTrigger().whileTrue(swerveDrive.zeroRobot());
    driver.getSwitchPieceTrigger().whileTrue(pieceTypeSwitcher.switchPieceSelected());
    op.getSwitchPieceTrigger().whileTrue(pieceTypeSwitcher.switchPieceSelected());
    // setup default commands that are used for driving
    swerveDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    leds.setDefaultCommand(new RainbowLeds(leds).ignoringDisable(true));
    // frontLeds.setDefaultCommand(
    //    new ShowTargetInfo(frontLeds, frontCamera, Color.fromHSV(75, 255, 255)));
    // rearLeds.setDefaultCommand(
    //    new ShowTargetInfo(rearLeds, frontCamera, Color.fromHSV(75, 255, 255)));
    // frontLeds.setDefaultCommand(new RainbowLeds(frontLeds));
    // rearLeds.setDefaultCommand(new RainbowLeds(rearLeds));
    // rearLeds.setDefaultCommand(new TestLeds(rearLeds));

    new Trigger(() -> Math.abs(op.getElevatorRequest()) > 0.03)
        .whileTrue(elevator.driveElevator(op::getElevatorRequest, pivot::getAngle));
    elevator.setDefaultCommand(elevator.holdElevator());
    new Trigger(() -> Math.abs(op.getPivotRequest()) > 0.03)
        .whileTrue(pivot.drivePivot(op::getPivotRequest, elevator::getPosition));
    pivot.setDefaultCommand(pivot.holdClawPivot());
    intake.setDefaultCommand(intake.driveIntake(op::getIntakeRequest, pieceTypeSwitcher::isCoral));
    new Trigger(() -> op.getL1Command() && pieceTypeSwitcher.isCoral())
        .whileTrue(setScoringPosition(ScoringPositions.L1Coral));
    new Trigger(() -> op.getL2Command() && pieceTypeSwitcher.isCoral())
        .whileTrue(setScoringPosition(ScoringPositions.L2Coral));
    new Trigger(() -> op.getL3Command() && pieceTypeSwitcher.isCoral())
        .whileTrue(setScoringPosition(ScoringPositions.L3Coral));
    new Trigger(() -> op.getL4Command() && pieceTypeSwitcher.isCoral())
        .whileTrue(setScoringPosition(ScoringPositions.L4Coral));
    new Trigger(() -> op.getL1Command() && pieceTypeSwitcher.isAlgae())
        .whileTrue(setScoringPosition(ScoringPositions.ProcessorAlgae));
    new Trigger(() -> op.getL2Command() && pieceTypeSwitcher.isAlgae())
        .whileTrue(setScoringPosition(ScoringPositions.L2Algae));
    new Trigger(() -> op.getL3Command() && pieceTypeSwitcher.isAlgae())
        .whileTrue(setScoringPosition(ScoringPositions.L3Algae));
    new Trigger(() -> op.getL4Command() && pieceTypeSwitcher.isAlgae())
        .whileTrue(setScoringPosition(ScoringPositions.NetAlgae));
    new Trigger(() -> op.getLollipopCommand() && pieceTypeSwitcher.isAlgae())
        .whileTrue(setScoringPosition(ScoringPositions.Lollipop));
    new Trigger(() -> op.getLoadingPositionCommand())
        .whileTrue(setScoringPosition(ScoringPositions.LoadingPosition));

    new Trigger(
            () -> {
              return elevator.getCollisionWarning() || pivot.getCollisionWarning();
            })
        .whileTrue(new LightningFlash(leds, Color.kRed).andThen(new BreathLeds(leds, Color.kRed)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  enum Zones {
    ZoneA,
    ZoneB,
    ZoneC,
    ZoneD
  }

  private Zones getZone(double height, double angle) {
    // Zone A - dist < 27.3 && angle < 15 (intake area)
    // Zone B - angle > 15 (everything on the scoring side)
    // Zone C - dist > 57.2 && angle < 15 (algae scoring)
    // Zone D - 27.3 < dist < 57.2 && angle < 15 (bad!!!)
    if (angle > 15.0) {
      return Zones.ZoneB;
    } else if (height < 27.3) {
      return Zones.ZoneA;
    } else if (height > 57.2) {
      return Zones.ZoneC;
    } else {
      return Zones.ZoneD;
    }
  }

  private Command setScoringPosition(ScoringPositions position) {
    return new DeferredCommand(() -> setScoringPositionDeferred(position), Set.of(elevator, pivot));
  }

  private Command setScoringPositionDeferred(ScoringPositions position) {
    Zones curZone = getZone(elevator.getPosition(), pivot.getAngle());
    Zones destZone = getZone(elevator.getSetPosition(position), pivot.getSetPosition(position));

    if (curZone == Zones.ZoneD) {
      //if we start in danger zone, get out, then rerun this logic to get to the spot
      return pivot.setAngleCmd(30).andThen(setScoringPosition(position));
    } else if (curZone == destZone) { // any zone to any zone
      return new ParallelCommandGroup(
          elevator.setPositionCmd(position).andThen(elevator.holdElevator()),
          pivot.setAngleCmd(position).andThen(pivot.holdClawPivot()));
    } else if ((curZone == Zones.ZoneA || curZone == Zones.ZoneC)
        && destZone == Zones.ZoneB) { // (A or C) to B
      return pivot
          .setAngleCmd(45.0)
          .until(() -> pivot.getAngle() > 20) // continue once we have cleared enough
          .andThen(
              new ParallelCommandGroup(
                  elevator.setPositionCmd(position).andThen(elevator.holdElevator()),
                  pivot.setAngleCmd(position).andThen(pivot.holdClawPivot())));
    } else if ((curZone == Zones.ZoneA && destZone == Zones.ZoneC)
        || (curZone == Zones.ZoneC
            && destZone
                == Zones.ZoneA)) { // A or C) to (A or C) [but not going from A to A or C to C]
      return pivot
          .setAngleCmd(45.0)
          .until(() -> pivot.getAngle() > 20) // continue once we have cleared enough
          .andThen(elevator.setPositionCmd(position))
          .andThen(pivot.setAngleCmd(position));
    } else if (curZone == Zones.ZoneB
        && (destZone == Zones.ZoneA || destZone == Zones.ZoneC)) { // B to (A or C)
      return elevator.setPositionCmd(position).andThen(pivot.setAngleCmd(position));
      // drive to low position, 20 in
      // move claw/destination to position
    } else { // scary...
      return new LightningFlash(leds, Color.kLavenderBlush)
          .andThen(new BreathLeds(leds, Color.kLavender));
    }
  }
}
