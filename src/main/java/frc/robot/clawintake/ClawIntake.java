package frc.robot.clawintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public abstract class ClawIntake extends SubsystemBase {
  abstract void setPower(double power);

  @AutoLogOutput
  public abstract boolean hasCoral();

  @AutoLogOutput
  public abstract boolean hasAlgae();

  public ClawIntake() {
    super();
    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {}

  public Command driveIntake(DoubleSupplier pct, BooleanSupplier isCoral) {
    return run(
        () -> {
          var speed = pct.getAsDouble();
          if (isCoral.getAsBoolean()) {
            speed *= -1;
          }
          setPower(speed);
        });
  }
}
