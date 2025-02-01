package frc.robot.clawpivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutputManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ClawPivot extends SubsystemBase {
  boolean pidEnabled;
  public abstract void setPower(double power);
  public abstract void setAngle(double angle);
  public abstract double getSensorAngle();
  public abstract double getAngle();
  public abstract void setEncoderPosition(double position);

  public ClawPivot() {
    super();
    AutoLogOutputManager.addObject(this);
    pidEnabled = false;
  }

  @Override
  public void periodic() {}

  public Command drivePivot(DoubleSupplier pct) {
    return run(() -> setPower(pct.getAsDouble()));
  }
}
