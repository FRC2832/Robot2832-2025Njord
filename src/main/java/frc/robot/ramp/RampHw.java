package frc.robot.ramp;

import com.ctre.phoenix6.hardware.CANcoder;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class RampHw extends Ramp {
  private CANcoder rampAngle;
  private TalonFXMotor rampMotor;

  public RampHw() {
    super();
    rampAngle = new CANcoder(53);
    rampMotor = new TalonFXMotor("Ramp", 51);
  }

  @Override
  public void setPower(double power) {
    rampMotor.setPower(power);
  }

  @Override
  public void setAngle(double angle) {}

  @Override
  public double getAngle() {
    return rampAngle.getPosition().getValueAsDouble();
  }
}
