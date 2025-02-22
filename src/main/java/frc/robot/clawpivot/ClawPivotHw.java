package frc.robot.clawpivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ClawPivotHw extends ClawPivot {
  private TalonFXMotor pivotMotor;
  private CANcoder pivotAngle;

  public ClawPivotHw() {
    super();
    pivotMotor = new TalonFXMotor("Pivot", 58);
    pivotAngle = new CANcoder(57);

    CANcoderConfiguration configuration = new CANcoderConfiguration();
    CANcoderConfigurator cfg;
    cfg = pivotAngle.getConfigurator();
    StatusCode statusCode = cfg.refresh(configuration);
    configuration.MagnetSensor.MagnetOffset = 0.109375;
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    statusCode = cfg.apply(configuration);
    // stop unused warning for now
    statusCode = (StatusCode) statusCode;

    pivotMotor.setSoftLimits(4, 180);
    pivotMotor.setScaleFactor(1 / 7.649);
    pivotMotor.setCurrentLimit(10);
    pivotMotor.setBrakeMode(true);

    pivotMotor.setEncoderPosition(getSensorAngle());
  }

  @Override
  void setPower(double power) {
    pivotMotor.setPower(power);
  }

  @Override
  void setAngle(double angle) {
    if (pidEnabled) {
      pivotMotor.setPosition(angle);
    } else {
      pivotMotor.stopMotor(false);
    }
  }

  @Override
  public double getSensorAngle() {
    return Rotations.of(pivotAngle.getPosition().getValueAsDouble()).in(Degrees);
  }

  @Override
  public double getAngle() {
    return pivotMotor.getPosition();
  }

  @Override
  void setEncoderPosition(double position) {
    pivotMotor.setEncoderPosition(position);
  }

  @Override
  TalonFX getMotor() {
    return (TalonFX) pivotMotor.getBaseMotor();
  }
}
