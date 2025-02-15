package frc.robot.elevator;

import au.grapplerobotics.LaserCan;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ElevatorHw extends Elevator {
  TalonFXMotor leftMotor;
  TalonFXMotor rightMotor;
  LaserCan distSensor;

  public ElevatorHw() {
    super();

    leftMotor = new TalonFXMotor("Left Elevator", 55, "rio");
    rightMotor = new TalonFXMotor("Right Elevator", 56, "rio", true);
    // leftMotor.setSoftLimits(16, 80);
    // leftMotor.setScaleFactor(1);
    leftMotor.setCurrentLimit(10);
    leftMotor.setBrakeMode(true);

    distSensor = new LaserCan(1);
    /*
    TalonFX leftKracken = (TalonFX)leftMotor.getBaseMotor();
    TalonFX rightKracken = (TalonFX)rightMotor.getBaseMotor();
    var follower = new Follower(leftKracken.getDeviceID(), true);
    rightKracken.setControl(follower);
    */
  }

  @Override
  public void setPosition(double distance) {
    if (pidEnabled) {
      leftMotor.setPosition(distance);
    }
  }

  @Override
  public void setPower(double pct) {
    leftMotor.setPower(pct);
  }

  @Override
  public double getPosition() {
    return leftMotor.getPosition();
  }

  @Override
  public double getDistanceSensor() {
    return distSensor.getMeasurement().distance_mm * 1000;
  }

  @Override
  public void setEncoderPosition(double position) {
    leftMotor.setEncoderPosition(position);
  }
}
