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
  private ChassisSpeeds noSpeeds;
  private double lastHeading;
  private Pose2d target;
  private int counts;
  private int maxCounts;

  public AlignToPose(SwerveSubsystem swerve, Pose2d pose) {
    this(swerve, pose, 10);
  }

  public AlignToPose(SwerveSubsystem swerve, Pose2d pose, int endCounts) {
    this.drive = swerve;
    this.target = pose;
    this.maxCounts = endCounts;
    noSpeeds = new ChassisSpeeds();

    xController = new PIDController(3.1, 0.04, 0); // Vertical movement
    yController = new PIDController(3.1, 0.08, 0); // Horitontal movement
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
    counts = 0;
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    var xError = Units.metersToInches(pose.getX() - target.getX());
    var yError = Units.metersToInches(pose.getY() - target.getY());
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

    // send the drive command
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);
    drive.driveFieldOriented(speeds);

    // calculate if we are at goal
    // error was 0.8" X, 1.1" Y, or 1.36" total
    double distError = Math.sqrt((xError * xError) + (yError * yError));
    if (Math.abs(rotError) < 2 && distError < 1.2) {
      counts++;
    } else {
      // either decrement the count or make it zero
      counts = Math.max(counts--, 0);
    }
  }

  @Override
  public boolean isFinished() {
    return counts >= maxCounts;
  }

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(noSpeeds);
  }
}
