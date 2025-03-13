package frc.robot.swervedrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShakeRobot extends Command {
  private SwerveSubsystem swerve;
  private double startTime;
  double lastChangeTime;
  boolean leftFirst;
  boolean switched;
  int speeeed;

  public ShakeRobot(SwerveSubsystem swerve, boolean leftFirst) {
    this.swerve = swerve;
    this.leftFirst = leftFirst;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if (leftFirst) {
      speeeed = 3;
    } else {
      speeeed = -3;
    }
    lastChangeTime = 0;
  }

  @Override
  public void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    if ((elapsedTime - lastChangeTime) > 0.333) {
      speeeed = speeeed * -1;
      lastChangeTime = elapsedTime;
    }
    swerve.drive(new ChassisSpeeds(0, speeeed, 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
