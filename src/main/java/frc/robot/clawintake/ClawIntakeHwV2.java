package frc.robot.clawintake;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.LinearFilter;
import org.livoniawarriors.motorcontrol.SparkFlexMotor;

public class ClawIntakeHwV2 extends ClawIntake {
  private SparkFlexMotor intakeMotor;
  private LaserCan pieceSensor;
  private double distance;
  private LinearFilter filter;

  public ClawIntakeHwV2() {
    super();
    intakeMotor = new SparkFlexMotor("Intake", 59);
    pieceSensor = new LaserCan(2);

    intakeMotor.pidConstants.kP = 0.000001;
    intakeMotor.pidConstants.kI = 0;
    intakeMotor.pidConstants.kV = 1. / 565.;
    intakeMotor.configurePid();

    // filter out the noisy distance sensor
    filter = LinearFilter.singlePoleIIR(0.06, 0.02);
  }

  @Override
  void setPower(double power) {
    intakeMotor.setPower(power);
  }

  @Override
  public boolean hasCoral() {
    return 0 <= distance && distance < 10;
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
  void setRpm(double rpm) {
    intakeMotor.setRpm(rpm);
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
}
