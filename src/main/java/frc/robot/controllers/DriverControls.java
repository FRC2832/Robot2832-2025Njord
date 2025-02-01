package frc.robot.controllers;

import edu.wpi.first.networktables.DoubleSubscriber;
import org.livoniawarriors.T16000M;
import org.livoniawarriors.UtilFunctions;

public class DriverControls {
  private T16000M driverLeft;
  private T16000M driverRight;
  private DoubleSubscriber deadband;

  public DriverControls() {
    driverLeft = new T16000M(0);
    driverRight = new T16000M(1);
    deadband = UtilFunctions.getSettingSub("DriveStick/Deadband", 0.02);
  }

  public double getDriveX() {
    var dead = deadband.get();
    return UtilFunctions.deadband(-driverRight.getyAxis1(), dead);
  }

  public double getDriveY() {
    var dead = deadband.get();
    return UtilFunctions.deadband(-driverRight.getxAxis1(), dead);
  }

  public double getTurn() {
    var dead = deadband.get();
    return -UtilFunctions.deadband(driverLeft.getxAxis1(), dead);
  }
}
