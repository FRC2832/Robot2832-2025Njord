package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public abstract class Climb extends SubsystemBase {
  public abstract void setPower(double power);

  public abstract void setAngle(double angle);

  @AutoLogOutput
  public abstract double getAngle();

  public Climb() {
    super();
    AutoLogOutputManager.addObject(this);
  }

  public Command setPowerCmd(DoubleSupplier pct) {
    return run(() -> setPower(pct.getAsDouble()));
  }
}
