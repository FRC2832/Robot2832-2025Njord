package frc.robot.elevator;

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

public abstract class Elevator extends SubsystemBase {
  public abstract void setPosition(double distance);

  public abstract void setPower(double pct);

  @AutoLogOutput
  public abstract double getPosition();

  public abstract double getDistanceSensor();

  public abstract void setEncoderPosition(double position);

  boolean pidEnabled;
  DoubleEntry heightPub;
  private HashMap<ScoringPositions, DoubleSupplier> positions;

  public Elevator() {
    super();
    AutoLogOutputManager.addObject(this);
    pidEnabled = false;
    SmartDashboard.putData("Set Elevator 20", setPositionCmd(20));
    SmartDashboard.putData("Set Elevator 70", setPositionCmd(70));
    heightPub =
        NetworkTableInstance.getDefault().getDoubleTopic("/Simulation/Claw Height").getEntry(0);
    positions = new HashMap<>();
    // these position numbers are completely made up
    positions.put(
        ScoringPositions.L1Coral, UtilFunctions.getSettingSub("ElevatorPos/L1Coral", 17.8));
    positions.put(
        ScoringPositions.L2Coral, UtilFunctions.getSettingSub("ElevatorPos/L2Coral", 24.7));
    positions.put(ScoringPositions.L3Coral, UtilFunctions.getSettingSub("ElevatorPos/L3Coral", 40));
    positions.put(ScoringPositions.L4Coral, UtilFunctions.getSettingSub("ElevatorPos/L4Coral", 69));
    positions.put(
        ScoringPositions.LoadingPosition,
        UtilFunctions.getSettingSub("ElevatorPos/LoadingPosition", 16.5));
    positions.put(ScoringPositions.L2Algae, UtilFunctions.getSettingSub("ElevatorPos/L2Algae", 36));
    positions.put(
        ScoringPositions.L3Algae, UtilFunctions.getSettingSub("ElevatorPos/L3Algae", 52.4));
    positions.put(
        ScoringPositions.ProcessorAlgae,
        UtilFunctions.getSettingSub("ElevatorPos/ProcessorAlgae", 16.5));
    positions.put(
        ScoringPositions.NetAlgae, UtilFunctions.getSettingSub("ElevatorPos/NetAlgae", 80));
    positions.put(
        ScoringPositions.Lollipop, UtilFunctions.getSettingSub("ElevatorPos/Lollipop", 22.7));
  }

  @Override
  public void periodic() {
    heightPub.set(getPosition());
  }

  public Command driveElevator(DoubleSupplier pct) {
    return run(() -> setPower(pct.getAsDouble()));
  }

  public Command setPositionCmd(double position) {
    return run(() -> setPosition(position));
  }

  public Command setPositionCmd(ScoringPositions position) {
    return run(() -> setPosition(getSetPosition(position)));
  }

  public double getSetPosition(ScoringPositions position) {
    return positions.get(position).getAsDouble();
  }
}
