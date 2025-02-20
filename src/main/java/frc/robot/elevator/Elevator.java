package frc.robot.elevator;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public abstract void setVoltage(double volts);

  boolean pidEnabled;
  DoubleEntry heightPub;

  public Elevator() {
    super();
    AutoLogOutputManager.addObject(this);
    pidEnabled = false;
    SmartDashboard.putData("Set Elevator 20", setPositionCmd(20));
    SmartDashboard.putData("Set Elevator 70", setPositionCmd(70));
    heightPub =
        NetworkTableInstance.getDefault().getDoubleTopic("/Simulation/Claw Height").getEntry(0);
  }

  @Override
  public void periodic() {
    heightPub.set(getPosition());
  }

  public Command driveElevator(DoubleSupplier pct) {
    return run(() -> setVoltage((pct.getAsDouble() * 12) + 0.5));
  }

  public Command setPositionCmd(double position) {
    return run(() -> setPosition(position));
  }


}
