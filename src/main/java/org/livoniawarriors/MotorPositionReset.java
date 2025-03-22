package org.livoniawarriors;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Robot;

public class MotorPositionReset {
  private double timer;

  private double SettleTime;
  private double SensorDifference;
  private double MotorVelocity;
  private double lastMotorPosition;
  private LinearFilter filter;

  public MotorPositionReset(double settleTime, double sensorDifference, double motorVelocity) {
    this.SettleTime = settleTime;
    this.SensorDifference = sensorDifference;
    this.MotorVelocity = motorVelocity;
    timer = 0;
    lastMotorPosition = 0;
    // filter out the noisy distance sensor
    filter = LinearFilter.singlePoleIIR(0.16, Robot.kDefaultPeriod);
  }

  /**
   * @return Returns when the reset is needed
   */
  public boolean updateReset(double motorPosition, double sensorPosition) {
    boolean incrementTimer = true;
    double lastTimer = timer;
    double velocity = (motorPosition - lastMotorPosition) / Robot.kDefaultPeriod;
    lastMotorPosition = motorPosition;

    if (Math.abs(motorPosition - sensorPosition) < SensorDifference) {
      incrementTimer = false;
    }

    if (Math.abs(filter.calculate(velocity)) > MotorVelocity) {
      incrementTimer = false;
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
