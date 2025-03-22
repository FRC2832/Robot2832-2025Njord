package frc.robot.clawintake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public abstract class ClawIntake extends SubsystemBase {
  abstract void setPower(double power);

  abstract void setVelocity(double velocity);

  abstract void setRpm(double rpm);

  abstract void setPosition(double position);

  abstract double getPosition();

  abstract double getVelocity();

  @AutoLogOutput
  public abstract boolean hasCoral();

  @AutoLogOutput
  public abstract boolean hasAlgae();

  abstract void updateSensors();

  public boolean holdCoral = false;

  public ClawIntake() {
    super();
    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    updateSensors();

    if (DriverStation.isDisabled() && hasCoral()) {
      holdCoral = true;
    } else if (hasCoral() == false && Math.abs(getVelocity()) < 0.5) {
      holdCoral = false;
    }
  }

  public Command driveIntake(DoubleSupplier pct, BooleanSupplier isCoral) {
    return run(
        () -> {
          // convert to max of 5000 rpm
          var speed = pct.getAsDouble() * 0.012 * 5000.;
          if (isCoral.getAsBoolean()) {
            speed *= -1;
          }
          setVelocity(speed);
        });
  }

  public Command driveIntakeFast(BooleanSupplier isCoral, DoubleSupplier clawAngle) {
    return run(
        () -> {
          var pct = 0.85;
          if (isCoral.getAsBoolean() && clawAngle.getAsDouble() < 100) {
            pct *= -1;
          }
          setPower(pct);
        });
  }

  public Trigger trigCoralHome(DoubleSupplier driveCommand, BooleanSupplier isCoral) {
    // if we see a coral and need to home
    return new Trigger(
        () -> (driveCommand.getAsDouble() > 0) && (isCoral.getAsBoolean()) && !holdCoral);
  }

  public Command homeCoral(DoubleSupplier driveCommand) {
    return new HomeCoral(this, driveCommand);
  }
}
