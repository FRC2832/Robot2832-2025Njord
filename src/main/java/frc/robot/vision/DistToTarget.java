package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.livoniawarriors.UtilFunctions;

public class DistToTarget implements Runnable {
  Supplier<Pose2d> targetSup;
  Supplier<Pose2d> currentLocSup;

  public DistToTarget(Supplier<Pose2d> target, Supplier<Pose2d> currentLoc) {
    this.targetSup = target;
    this.currentLocSup = currentLoc;
  }

  @Override
  public void run() {
    var target = targetSup.get();
    var currentLoc = currentLocSup.get();

    double angleErrorRad = currentLoc.getRotation().minus(target.getRotation()).getRadians();
    double dist = UtilFunctions.getDistance(target, currentLoc);
    double distX = dist * Math.cos(angleErrorRad);
    double distY = dist * Math.sin(angleErrorRad);

    SmartDashboard.putNumber("Pole DistX", Units.metersToInches(distX));
    SmartDashboard.putNumber("Pole DistY", Units.metersToInches(distY));
    SmartDashboard.putNumber("Pole DistRot", Math.toDegrees(angleErrorRad));
  }
}
