// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.leds.TestLeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.leds.FrontLeds;
import frc.robot.leds.RearLeds;
import frc.robot.leds.ShowTargetInfo;
import frc.robot.ramp.RampSubsystem;
import frc.robot.swervedrive.SwerveSubsystem;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private SwerveSubsystem swerveDrive;
    private RampSubsystem rampSubsystem;
    private FrontLeds frontLeds;
    private RearLeds rearLeds;
    private Vision vision;

    private XboxController driverController;

    private SendableChooser<Command> autoChooser;

    private AprilTagCamera frontCamera;

    public RobotContainer() {
        driverController = new XboxController(0);

        String swerveDirectory = "swerve/kitbot";
        //subsystems used in all robots
        swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveDirectory));
        frontLeds = new FrontLeds(6, 54);
        rearLeds = new RearLeds(frontLeds);
        rampSubsystem = new RampSubsystem();
        if(Robot.isSimulation()) {
            //drive fast in simulation
            swerveDrive.setMaximumSpeed(5, Math.PI);
            //start in red zone since simulation defaults to red 1 station to make field oriented easier
            swerveDrive.resetOdometry(new Pose2d(16.28, 4.03,Rotation2d.fromDegrees(180)));
        }
        else {
            swerveDrive.setMaximumSpeed(1, Math.PI/2);
        }

        vision = new Vision(swerveDrive);
        frontCamera = new AprilTagCamera("front",
            new Rotation3d(0, Units.degreesToRadians(0), Math.toRadians(0)),
            new Translation3d(0.363,
                                0,
                                0.31),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

        vision.addCamera(frontCamera);
        /*
        vision.addCamera(new AprilTagCamera("rear",
            new Rotation3d(0, Units.degreesToRadians(-20), Math.toRadians(0)),
            new Translation3d(-0.363,
                                0,
                                0.5),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)));
        */
        //add some buttons to press for development
        /*
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.WheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDrive, MoveWheels.WheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.WheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsDiamond()));
        */
        //SmartDashboard.putData("Test Leds", new TestLeds(leds));

        // Register Named Commands for PathPlanner
        //NamedCommands.registerCommand("flashRed", new LightningFlash(leds, Color.kFirstRed));
        //NamedCommands.registerCommand("flashBlue", new LightningFlash(leds, Color.kFirstBlue));
        NamedCommands.registerCommand("ScorePieceL1", new WaitCommand(1));
        NamedCommands.registerCommand("GetFromHP", new WaitCommand(2));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = swerveDrive.driveCommand(
            () -> MathUtil.applyDeadband(driverController.getLeftY() * -1, 0.05),
            () -> MathUtil.applyDeadband(driverController.getLeftX() * -1, 0.05),
            () -> driverController.getRightX() * -1);

        if (Robot.isSimulation()) {
            new Trigger(driverController::getAButton).whileTrue(swerveDrive.driveToPose(new Pose2d(11.23, 4.15, Rotation2d.fromDegrees(0))));
            new Trigger(driverController::getYButton).whileTrue(swerveDrive.driveToPose(new Pose2d(14.73, 4.49, Rotation2d.fromDegrees(120))));
        } else {
            new Trigger(driverController::getAButton).whileTrue(swerveDrive.driveToPose(new Pose2d(2.75, 4.15, Rotation2d.fromDegrees(0))));
        }
        new Trigger(driverController::getLeftStickButton).whileTrue(swerveDrive.swerveLock());
        
        //setup default commands that are used for driving
        swerveDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        //leds.setDefaultCommand(new RainbowLeds(leds).ignoringDisable(true));
        frontLeds.setDefaultCommand(new ShowTargetInfo(frontLeds, frontCamera, Color.fromHSV(75, 255, 255)));
        rearLeds.setDefaultCommand(new ShowTargetInfo(rearLeds, frontCamera, Color.fromHSV(75, 255, 255)));
        //rearLeds.setDefaultCommand(new TestLeds(rearLeds));
        rampSubsystem.setDefaultCommand(rampSubsystem.runMotor(() -> (driverController.getRightTriggerAxis() * 0.35) - (driverController.getLeftTriggerAxis() * 0.35)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
