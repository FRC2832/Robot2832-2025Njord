package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public abstract class Elevator extends SubsystemBase {
  public abstract void setPosition(double distance);

  public abstract void setPower(double pct);

  @AutoLogOutput
  public abstract double getPosition();

  public abstract double getDistanceSensor();

  public abstract void setEncoderPosition(double position);

  boolean pidEnabled;

  public Elevator() {
    super();
    AutoLogOutputManager.addObject(this);
    pidEnabled = false;
  }

  @Override
  public void periodic() {}

  public Command driveElevator(DoubleSupplier pct) {
    return run(() -> setPower(pct.getAsDouble()));
  }
}
