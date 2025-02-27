package org.livoniawarriors;

import frc.robot.Robot;

public class MotorPositionReset {
  private double timer;

  private double SettleTime;
  private double SensorDifference;

  public MotorPositionReset(double settleTime, double sensorDifference) {
    this.SettleTime = settleTime;
    this.SensorDifference = sensorDifference;
    timer = 0;
  }

  /**
   * @return Returns when the reset is needed
   */
  public boolean updateReset(double motorPosition, double sensorPosition) {
    boolean incrementTimer = false;
    double lastTimer = timer;

    if (Math.abs(motorPosition - sensorPosition) > SensorDifference) {
      incrementTimer = true;
    }

    if (incrementTimer) {
      timer = Math.min(timer + Robot.kDefaultPeriod, SettleTime);
    } else {
      timer = Math.max(timer - Robot.kDefaultPeriod, 0);
    }

    if (timer >= SettleTime && timer != lastTimer) {
      return true;
    }
    return false;
  }

  public void resetTimer() {
    timer = 0;
  }
}
