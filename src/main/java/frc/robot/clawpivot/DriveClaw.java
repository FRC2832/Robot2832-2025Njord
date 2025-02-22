package frc.robot.clawpivot;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class DriveClaw extends Command {
  private ClawPivot clawPivot;
  private DoubleSupplier elevatorHDoubleSupplier;
  private DoubleSupplier driverRequest;
  final double SAFETY_BUFFER = 6;
  final double ANGLE_SAFETY = 10;

  public DriveClaw(
      ClawPivot clawPivot, DoubleSupplier elevatorHDoubleSupplier, DoubleSupplier driverRequest) {
    this.clawPivot = clawPivot;
    this.elevatorHDoubleSupplier = elevatorHDoubleSupplier;
    this.driverRequest = driverRequest;
    addRequirements(clawPivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean canMove;
    double elevatorPos = elevatorHDoubleSupplier.getAsDouble();
    double clawAngle = clawPivot.getAngle();
    double request = driverRequest.getAsDouble();

    if ((27.3 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 36.5 + SAFETY_BUFFER)
        && (clawAngle < 15 + ANGLE_SAFETY)) {
      // its too close here, allow if moving out
      canMove = request > 0;
    } else if ((43.7 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 57.2 + SAFETY_BUFFER)
        && (clawAngle < 15 + ANGLE_SAFETY)) {
      // its too close here
      canMove = request > 0;
    } else if ((0 - SAFETY_BUFFER < elevatorPos)
        && (elevatorPos < 19 + SAFETY_BUFFER)
        && (clawAngle > 120 - ANGLE_SAFETY)) {
      // its too close here
      canMove = request < 0;
    } else {
      // its all good!!! keep moving and such and such and so on
      canMove = true;
    }

    if (canMove) {
      clawPivot.setPower(driverRequest.getAsDouble());
    } else {
      // TODO replace with stop?
      clawPivot.setPower(0);
    }
    clawPivot.setCollisionWarning(!canMove);

    // range1 = 27.3 - 36.5", 15* out
    // range2 = 43.7 - 57.2", 15* out
    // range3 = 0 - 19", 120* out
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    clawPivot.setCollisionWarning(false);
  }
}
