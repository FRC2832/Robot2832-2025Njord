package frc.robot.clawintake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ClawIntakeHw extends ClawIntake {
  private TalonFXMotor intakeMotor;
  private LaserCan pieceSensor;

  public ClawIntakeHw() {
    super();
    intakeMotor = new TalonFXMotor("Intake", 59);
    pieceSensor = new LaserCan(2);
  }

  @Override
  void setPower(double power) {
    intakeMotor.setPower(power);
  }

  @Override
  public boolean hasCoral() {
    return pieceSensor.getMeasurement().distance_mm
        < Distance.ofBaseUnits(4, Inches).in(Millimeters);
  }

  @Override
  public boolean hasAlgae() {
    return false;
  }
}
