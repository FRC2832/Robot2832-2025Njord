package frc.robot.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToPose extends Command {
  private PIDController xController, yController, rotController;
  private SwerveSubsystem drive;
  private boolean atSetpoint;
  private ChassisSpeeds noSpeeds;
  private double lastHeading;
  private Pose2d target;
  private int counts;

  public AlignToPose(SwerveSubsystem swerve, Pose2d pose) {
    this.drive = swerve;
    this.target = pose;
    noSpeeds = new ChassisSpeeds();

    xController = new PIDController(3.1, 0.02, 0); // Vertical movement
    yController = new PIDController(3.1, 0.02, 0); // Horitontal movement
    rotController = new PIDController(0.05, 0.0004, 0); // Rotation

    rotController.setSetpoint(pose.getRotation().getDegrees());
    rotController.setTolerance(4);
    lastHeading = pose.getRotation().getDegrees();

    xController.setSetpoint(pose.getX());
    xController.setTolerance(Units.inchesToMeters(0.9));

    yController.setSetpoint(pose.getY());
    yController.setTolerance(Units.inchesToMeters(0.9));
  }

  @Override
  public void initialize() {
    atSetpoint = false;
    counts = 0;
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    var xError = Units.inchesToMeters(pose.getX() - target.getX());
    var yError = Units.inchesToMeters(pose.getY() - target.getY());
    var rotError = pose.getRotation().minus(target.getRotation()).getDegrees();

    double xSpeed = xController.calculate(pose.getX());
    double ySpeed = yController.calculate(pose.getY());
    SmartDashboard.putNumber("Align XError", xError);
    SmartDashboard.putNumber("Align YError", yError);
    SmartDashboard.putNumber("Align RotError", rotError);

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

    if (Math.abs(rotError) < 2 && Math.abs(xError) < 0.2 && Math.abs(yError) < 0.2) {
      atSetpoint = true;
      counts++;
    } else {
      atSetpoint = false;
      counts = 0;
    }

    drive.driveFieldOriented(speeds);
  }

  @Override
  public boolean isFinished() {
    return counts >= 25;
  }

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(noSpeeds);
  }
}
