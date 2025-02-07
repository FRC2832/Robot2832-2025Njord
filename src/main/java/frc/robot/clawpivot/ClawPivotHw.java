package frc.robot.clawpivot;

import com.ctre.phoenix6.hardware.CANcoder;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ClawPivotHw extends ClawPivot {
  private TalonFXMotor pivotMotor;
  private CANcoder pivotAngle;

  public ClawPivotHw() {
    super();
    pivotMotor = new TalonFXMotor("Pivot", 7);
    pivotAngle = new CANcoder(2);

    // pivotMotor.setSoftLimits(16, 80);
    // pivotMotor.setScaleFactor(1);
    pivotMotor.setCurrentLimit(10);
    pivotMotor.setBrakeMode(true);
  }

  @Override
  void setPower(double power) {
    pivotMotor.setPower(power);
  }

  @Override
  void setAngle(double angle) {
    if (pidEnabled) {
      pivotMotor.setPosition(angle);
    }
  }

  @Override
  public double getSensorAngle() {
    return pivotAngle.getPosition().getValueAsDouble();
  }

  @Override
  public double getAngle() {
    return pivotMotor.getPosition();
  }

  @Override
  void setEncoderPosition(double position) {
    pivotMotor.setEncoderPosition(position);
  }
}
