package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class DriveElevator extends Command {
  private Elevator elevator;
  private DoubleSupplier clawAngleSup;
  private DoubleSupplier driverRequest;
  final double SAFETY_BUFFER = 2;
  final double ANGLE_SAFETY = 2;

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
    boolean canMove;
    double elevatorPos = elevator.getMotorPosition();
    double clawAngle = clawAngleSup.getAsDouble();
    double request = driverRequest.getAsDouble();
    if ((27.3 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 36.5 + SAFETY_BUFFER)
        && (clawAngle < 15 + ANGLE_SAFETY)) {
      // hold elevator at position
      //  if this and the next if-statement are true, this allows it to drive up into the point that
      // the second if-statement stops it from hitting(although thats because the safety buffers
      // interesct, i think??? once it moves outside of this if-statement's region it should be
      // good)
      // canMove = (request < 0) ^ (elevatorPos > 31.9);
      canMove = false;
    } else if ((43.7 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 57.2 + SAFETY_BUFFER)
        && (clawAngle < 15 + ANGLE_SAFETY)) {
      canMove = false;
    } else if ((0 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 19 + SAFETY_BUFFER)
        && (clawAngle
            > 120 - ANGLE_SAFETY)) { // the angle this turns true at is way too low, increase pls -
      // Luc
      canMove = request > 0;
    } else {
      canMove = true;
    }
    if (canMove) {
      elevator.setVoltage((driverRequest.getAsDouble() * 12));
    } else {
      elevator.setVoltage(Elevator.kG);
    }
    elevator.setCollisionWarning(!canMove);
    // range1 = 27.3 - 36.5", 15* out
    // range2 = 43.7 - 57.2", 15* out
    // range3 = 0 - 19", 120* out
    // elevator.setVoltage(elevator.kG);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setCollisionWarning(false);
  }
}
