package frc.robot.clawpivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public abstract class ClawPivot extends SubsystemBase {
  boolean pidEnabled;

  abstract void setPower(double power);

  abstract void setAngle(double angle);

  @AutoLogOutput
  public abstract double getSensorAngle();

  @AutoLogOutput
  public abstract double getAngle();

  abstract void setEncoderPosition(double position);

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
