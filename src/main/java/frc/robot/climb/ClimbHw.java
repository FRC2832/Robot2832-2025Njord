package frc.robot.climb;

import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ClimbHw extends Climb {
  // private CANcoder climbAngle;
  private TalonFXMotor climbMotor;

  public ClimbHw() {
    super();
    // climbAngle = new CANcoder(54);
    climbMotor = new TalonFXMotor("Climb", 50);
  }

  @Override
  public void setPower(double power) {
    climbMotor.setPower(power);
  }

  @Override
  public void setAngle(double angle) {}

  @Override
  public double getAngle() {
    // return climbAngle.getPosition().getValueAsDouble();
    return 0;
  }
}
