package frc.robot.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ClimbHw extends Climb {
  private CANcoder climbAngle;
  private TalonFXMotor climbMotor;
  TalonFX rawMotor;
  PositionVoltage positionSetter;

  public ClimbHw() {
    super();
    climbAngle = new CANcoder(54);
    climbMotor = new TalonFXMotor("Climb", 50);
    climbMotor.setBrakeMode(true);

    var fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = climbAngle.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.28;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    fx_cfg.Slot0.kP = 25;
    fx_cfg.Slot0.kG = 0.5;

    rawMotor = (TalonFX) climbMotor.getBaseMotor();
    rawMotor.getConfigurator().apply(fx_cfg);
    rawMotor.setNeutralMode(NeutralModeValue.Brake);

    positionSetter = new PositionVoltage(0).withUpdateFreqHz(1000);
    // climbMotor.pidConstants.kP = 50;
    // climbMotor.configurePid();
  }

  @Override
  public void setPower(double power) {
    // climbMotor.setPower(power);
    if (getAngle() < 0.24 || power < 0) {
      climbMotor.setPower(power);
    } else {
      rawMotor.setControl(positionSetter.withPosition(0.27));
    }
  }

  @Override
  public void setAngle(double angle) {}

  @Override
  public double getAngle() {
    return climbAngle.getPosition().getValueAsDouble();
  }
}
