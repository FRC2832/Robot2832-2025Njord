package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverFeedback implements Runnable {

  @Override
  public void run() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }
}
