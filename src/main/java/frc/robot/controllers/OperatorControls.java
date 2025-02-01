package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorControls {
  private XboxController operator;

  public OperatorControls() {
    operator = new XboxController(2);
  }

  public double getElevatorRequest() {
    return -operator.getLeftY();
  }

  public double getPivotRequest() {
    return -operator.getRightY();
  }

  public double getIntakeRequest() {
    return operator.getRightTriggerAxis()-operator.getLeftTriggerAxis();
  }
}
