package frc.robot.controllers;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.livoniawarriors.UtilFunctions;

public class OperatorControls {
  private XboxController operator;
  private DoubleSubscriber opDeadband;

  public OperatorControls() {
    operator = new XboxController(2);
    opDeadband = UtilFunctions.getSettingSub("/Operator/Deadband", 0.05);
  }

  public double getElevatorRequest() {
    return -UtilFunctions.deadband(operator.getLeftY(), opDeadband.get()) * 0.25;
  }

  public double getPivotRequest() {
    return -UtilFunctions.deadband(operator.getRightY(), opDeadband.get()) * 0.25;
  }

  public double getIntakeRequest() {
    return (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
  }

  public boolean getSwitchPiece() {
    return operator.getRightStickButton();
  }

  public boolean getL1Command() {
    return operator.getAButton();
  }

  public boolean getL2Command() {
    return operator.getBButton();
  }

  public boolean getL3Command() {
    return operator.getXButton();
  }

  public boolean getL4Command() {
    return operator.getYButton();
  }

  public boolean getLoadingPositionCommand() {
    return operator.getRightBumperButton();
  }

  public boolean getLollipopCommand() {
    return operator.getLeftBumperButton();
  }

  public Trigger getSwitchPieceTrigger() {
    return new Trigger(this::getSwitchPiece);
  }
}
