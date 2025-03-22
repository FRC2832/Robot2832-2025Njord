package frc.robot.clawintake;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.LinearFilter;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ClawIntakeHw extends ClawIntake {
  private TalonFXMotor intakeMotor;
  private LaserCan pieceSensor;
  private double distance;
  private LinearFilter filter;

  public ClawIntakeHw() {
    super();
    intakeMotor = new TalonFXMotor("Intake", 59);
    pieceSensor = new LaserCan(2);

    // filter out the noisy distance sensor
    filter = LinearFilter.singlePoleIIR(0.06, 0.02);
  }

  @Override
  void setPower(double power) {
    intakeMotor.setPower(power);
  }

  @Override
  public boolean hasCoral() {
    return 0 <= distance && distance < 40;
  }

  @Override
  public boolean hasAlgae() {
    return false;
  }

  @Override
  void setVelocity(double velocity) {
    intakeMotor.setVelocity(velocity);
  }

  @Override
  void setPosition(double position) {
    intakeMotor.setPosition(position);
  }

  @Override
  double getPosition() {
    return intakeMotor.getPosition();
  }

  @Override
  void updateSensors() {
    if (pieceSensor.getMeasurement() != null) {
      var measurement = pieceSensor.getMeasurement().distance_mm;
      distance = filter.calculate(measurement);
    }
  }

  @Override
  double getVelocity() {
    return intakeMotor.getVelocity();
  }

  @Override
  void setRpm(double rpm) {
    intakeMotor.setRpm(rpm);
  }
}
