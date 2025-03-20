package frc.robot.controllers;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  public boolean getSwerveLock() {
    return driverLeft.getLeft();
  }

  public Trigger getSwerveLockTrigger() {
    return new Trigger(this::getSwerveLock);
  }

  public boolean isFieldOrientedResetRequested() {
    return driverRight.getMiddle();
  }

  public Trigger isFieldOrientedResetRequestedTrigger() {
    return new Trigger(this::isFieldOrientedResetRequested);
  }

  public boolean getSwitchPiece() {
    return driverLeft.getRight();
  }

  public Trigger getSwitchPieceTrigger() {
    return new Trigger(this::getSwitchPiece);
  }

  public Trigger driveToPole() {
    return new Trigger(driverRight::getTrigger);
  }

  public double getClimbPercent() {
    int pov = driverLeft.getPOV();
    double request;
    if (pov == 0 || pov == 315 || pov == 45) {
      request = -0.37;
    } else if (pov == 180 || pov == 225 || pov == 135) {
      request = 0.37;
    } else {
      request = 0;
    }
    return request;
  }

  public Trigger getClimbRequest() {
    return new Trigger(
        () -> {
          return Math.abs(getClimbPercent()) > 0.02;
        });
  }

  public double getRampPercent() {
    int pov = driverRight.getPOV();
    double request;
    if (pov == 0 || pov == 315 || pov == 45) {
      request = 0.25;
    } else if (pov == 180 || pov == 225 || pov == 135) {
      request = -0.15;
    } else {
      request = 0;
    }
    return request;
  }

  public Trigger getRampRequest() {
    return new Trigger(
        () -> {
          return Math.abs(getRampPercent()) > 0.02;
        });
  }
}
