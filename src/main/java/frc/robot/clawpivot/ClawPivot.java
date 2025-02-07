package frc.robot.clawpivot;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  DoubleEntry clawPub;

  public ClawPivot() {
    super();
    AutoLogOutputManager.addObject(this);
    pidEnabled = false;

    SmartDashboard.putData("Set Claw 20*", setAngleCmd(20.));
    SmartDashboard.putData("Set Claw 170*", setAngleCmd(170.));
    clawPub =
        NetworkTableInstance.getDefault().getDoubleTopic("/Simulation/Claw Angle").getEntry(0);
  }

  @Override
  public void periodic() {
    clawPub.set(getAngle());
  }

  public Command drivePivot(DoubleSupplier pct) {
    return run(() -> setPower(pct.getAsDouble()));
  }

  public Command setAngleCmd(Double angle) {
    return run(() -> setAngle(angle));
  }
}
