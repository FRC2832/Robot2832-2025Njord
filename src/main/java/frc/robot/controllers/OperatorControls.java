package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    return operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
  }

  public boolean getSwitchPiece(){
    return operator.getRightStickButton();
  }

  public Trigger getSwitchPieceTrigger(){
    return new Trigger(this::getSwitchPiece);
  }
}
