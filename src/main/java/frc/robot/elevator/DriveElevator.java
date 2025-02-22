package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class DriveElevator extends Command {
  private Elevator elevator;
  private DoubleSupplier clawAngleSup;
  private DoubleSupplier driverRequest;
  final double SAFETY_BUFFER = 6;
  final double ANGLE_SAFETY = 10;

  public DriveElevator(
      Elevator elevator, DoubleSupplier clawAngleSup, DoubleSupplier driverRequest) {
    this.elevator = elevator;
    this.clawAngleSup = clawAngleSup;
    this.driverRequest = driverRequest;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double elevatorPos = elevator.getMotorPosition();
    double clawAngle = clawAngleSup.getAsDouble();
    if ((27.3 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 36.5 + SAFETY_BUFFER)
        && (clawAngle < 15 + ANGLE_SAFETY)) {
      // hold elevator at position
      elevator.setVoltage(elevator.kG);
      // elevator.setCollisionWarning(true)
    } else if ((43.7 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 57.2 + SAFETY_BUFFER)
        && (clawAngle < 15 + ANGLE_SAFETY)) {
      elevator.setVoltage(elevator.kG);
    } else if ((0 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 19 + SAFETY_BUFFER)
        && (clawAngle > 120 - ANGLE_SAFETY)) {
      elevator.setVoltage(elevator.kG);
    } else {
      elevator.setVoltage((driverRequest.getAsDouble() * 12) + elevator.kG);
    }
    // range1 = 27.3 - 36.5", 15* out
    // range2 = 43.7 - 57.2", 15* out
    // range3 = 0 - 19", 120* out
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
