package frc.robot.elevator;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ElevatorHw extends Elevator {
  TalonFXMotor leftMotor;
  TalonFXMotor rightMotor;
  LaserCan distSensor;
  private LinearFilter filter;

  public ElevatorHw() {
    super();

    leftMotor = new TalonFXMotor("Left Elevator", 55, "rio");
    rightMotor = new TalonFXMotor("Right Elevator", 56, "rio", true);
    leftMotor.setSoftLimits(20, 76);
    // leftMotor.setScaleFactor(1);
    leftMotor.setCurrentLimit(10);
    rightMotor.setCurrentLimit(10);
    leftMotor.setBrakeMode(true);

    distSensor = new LaserCan(1);
    // filter out the noisy distance sensor
    filter = LinearFilter.singlePoleIIR(0.16, 0.02);

    TalonFX leftKracken = (TalonFX) leftMotor.getBaseMotor();
    TalonFX rightKracken = (TalonFX) rightMotor.getBaseMotor();
    var follower = new Follower(leftKracken.getDeviceID(), false);
    rightKracken.setControl(follower);
  }

  @Override
  public void setPosition(double distance) {
    if (pidEnabled) {
      leftMotor.setPosition(distance);
    } else {
      leftMotor.stopMotor(false);
    }
  }

  @Override
  public void setPower(double pct) {
    leftMotor.setPower(pct);
  }

  @Override
  public double getMotorPosition() {
    return leftMotor.getPosition();
  }

  @Override
  public double getDistanceSensor() {
    return sensorValue;
    // return 0.133;
  }

  @Override
  public void setEncoderPosition(double position) {
    leftMotor.setEncoderPosition(position);
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }

  double sensorValue = 0.133;

  @Override
  void updateSensor() {
    if (distSensor.getMeasurement() != null) {
      double measurement = distSensor.getMeasurement().distance_mm / 1000.;
      sensorValue = filter.calculate(measurement);
    }
  }

  @Override
  protected double getVelocity() {
    return leftMotor.getVelocity();
  }
}
