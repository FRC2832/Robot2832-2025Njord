package frc.robot.clawpivot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.piecetypeswitcher.ScoringPositions;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.livoniawarriors.UtilFunctions;

public abstract class ClawPivot extends SubsystemBase {
  boolean pidEnabled;

  abstract void setPower(double power);

  abstract void setAngle(double angle);

  abstract TalonFX getMotor();

  @AutoLogOutput
  public abstract double getSensorAngle();

  @AutoLogOutput
  public abstract double getAngle();

  abstract void setEncoderPosition(double position);

  DoubleEntry clawPub;

  BooleanSubscriber pidEnableNtSub;
  boolean collisionWarning;

  private HashMap<ScoringPositions, DoubleSupplier> positions;

  public ClawPivot() {
    super();
    AutoLogOutputManager.addObject(this);
    pidEnabled = false;

    SmartDashboard.putData("Set Claw 20*", setAngleCmd(20.));
    SmartDashboard.putData("Set Claw 170*", setAngleCmd(170.));
    clawPub =
        NetworkTableInstance.getDefault().getDoubleTopic("/Simulation/Claw Angle").getEntry(0);
    positions = new HashMap<>();
    // these position numbers are completely made up
    positions.put(ScoringPositions.L1Coral, UtilFunctions.getSettingSub("ClawPos/L1Coral", 20));
    positions.put(ScoringPositions.L2Coral, UtilFunctions.getSettingSub("ClawPos/L2Coral", 24.7));
    positions.put(ScoringPositions.L3Coral, UtilFunctions.getSettingSub("ClawPos/L3Coral", 24.7));
    positions.put(ScoringPositions.L4Coral, UtilFunctions.getSettingSub("ClawPos/L4Coral", 60));
    positions.put(
        ScoringPositions.LoadingPosition,
        UtilFunctions.getSettingSub("ClawPos/LoadingPosition", 0));
    positions.put(ScoringPositions.L2Algae, UtilFunctions.getSettingSub("ClawPos/L2Algae", 179));
    positions.put(ScoringPositions.L3Algae, UtilFunctions.getSettingSub("ClawPos/L3Algae", 179));
    positions.put(
        ScoringPositions.ProcessorAlgae,
        UtilFunctions.getSettingSub("ClawPos/ProcessorAlgae", 179));
    positions.put(ScoringPositions.NetAlgae, UtilFunctions.getSettingSub("ClawPos/NetAlgae", 135));
    positions.put(ScoringPositions.Lollipop, UtilFunctions.getSettingSub("ClawPos/Lollipop", 185));
    pidEnableNtSub = UtilFunctions.getSettingSub("ClawPivot/EnablePid", false);
    collisionWarning = false;
  }

  @Override
  public void periodic() {
    clawPub.set(getAngle());
    pidEnabled = pidEnableNtSub.get();
  }

  public Command drivePivot(DoubleSupplier pct, DoubleSupplier elevatorHeight) {
    return new DriveClaw(this, elevatorHeight, pct);
  }

  public Command setAngleCmd(Double angle) {
    return run(() -> setAngle(angle));
  }

  public Command setAngleCmd(ScoringPositions position) {
    return run(() -> setAngle(getSetPosition(position)));
  }

  public double getSetPosition(ScoringPositions position) {
    return positions.get(position).getAsDouble();
  }

  public Command holdClawPivot() {
    return new HoldClawPivot(this);
  }

  public void setCollisionWarning(boolean warningOn) {
    collisionWarning = warningOn;
  }

  public boolean getCollisionWarning() {
    return collisionWarning;
  }
}
