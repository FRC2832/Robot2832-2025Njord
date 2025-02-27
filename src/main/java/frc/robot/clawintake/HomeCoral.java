package frc.robot.clawintake;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class HomeCoral extends Command {
  private ClawIntake intake;
  private double startPos;
  private DoubleSupplier driverInput;
  private Stages stage;

  enum Stages {
    FindPiece,
    MoveToEnd,
    DriveBack,
    Stop
  }

  public HomeCoral(ClawIntake intake, DoubleSupplier driverInput) {
    this.intake = intake;
    this.driverInput = driverInput;
    addRequirements(intake);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    startPos = intake.getPosition();
    startPos = (double) startPos;
    stage = Stages.FindPiece;
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  @Override
  public void execute() {
    // coral intake is negative, so go a negative amount of revolutions
    if (stage == Stages.FindPiece) {
      intake.setVelocity(-10);
      if (intake.hasCoral()) {
        stage = Stages.MoveToEnd;
      }
    } else if (stage == Stages.MoveToEnd) {
      intake.setVelocity(-10);
      if (!intake.hasCoral()) {
        stage = Stages.DriveBack;
        startPos = intake.getPosition();
      }
    } else if (stage == Stages.DriveBack) {
      // overshoot a little for the PID to react
      intake.setPosition(startPos + 5);
      if (intake.getPosition() > (startPos + 3)) {
        stage = Stages.Stop;
      }
    } else {
      // should never get here...
      intake.setVelocity(0);
    }
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.
   *
   * <p>Do not schedule commands here that share requirements with this command. Use {@link
   * #andThen(Command...)} instead.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    intake.setVelocity(0);
    if (stage != Stages.FindPiece) {
      intake.holdCoral = true;
    }
  }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  @Override
  public boolean isFinished() {
    if (driverInput.getAsDouble() > 0) {
      return false;
    }
    return stage == Stages.Stop;
    // return intake.getPosition() < (startPos - 12) || !intake.hasCoral();
  }
}
