package frc.robot.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToPose extends Command {
  private PIDController xController, yController, rotController;
  private SwerveSubsystem drive;
  private boolean atSetpoint;
  private ChassisSpeeds noSpeeds;
  private double lastHeading;

  public AlignToPose(SwerveSubsystem swerve, Pose2d pose) {
    this.drive = swerve;
    noSpeeds = new ChassisSpeeds();

    xController = new PIDController(3, 0, 0); // Vertical movement
    yController = new PIDController(3, 0, 0); // Horitontal movement
    rotController = new PIDController(0.04, 0, 0); // Rotation

    rotController.setSetpoint(pose.getRotation().getDegrees());
    rotController.setTolerance(1);
    lastHeading = pose.getRotation().getDegrees();

    xController.setSetpoint(pose.getX());
    xController.setTolerance(Units.inchesToMeters(0.5));

    yController.setSetpoint(pose.getY());
    yController.setTolerance(Units.inchesToMeters(0.5));
  }

  @Override
  public void initialize() {
    atSetpoint = false;
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    double xSpeed = xController.calculate(pose.getX());
    double ySpeed = yController.calculate(pose.getY());

    // handle 360 circle problem
    double curHeading = pose.getRotation().getDegrees();
    double centeredHeading =
        MathUtil.inputModulus(curHeading, lastHeading - 180, lastHeading + 180);
    double rotValue = rotController.calculate(centeredHeading);
    lastHeading = centeredHeading;

    // handle field oriented
    /*if (UtilFunctions.getAlliance() == Alliance.Red) {
      xSpeed *= -1;
      ySpeed *= -1;
    }*/
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);

    if (rotController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint()) {
      atSetpoint = true;
    }

    if (!atSetpoint) {
      drive.driveFieldOriented(speeds);
    } else {
      drive.driveFieldOriented(noSpeeds);
    }
  }

  @Override
  public boolean isFinished() {
    return atSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(noSpeeds);
  }
}
