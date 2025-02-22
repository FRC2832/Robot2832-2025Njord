package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.livoniawarriors.SparkFunDisplay;

public class DriverFeedback implements Runnable {
  SparkFunDisplay display;

  public DriverFeedback() {
    display = new SparkFunDisplay();
  }

  @Override
  public void run() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    display.display("RONY");
  }
}
