package frc.robot.clawintake;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldPiece extends Command {
  private ClawIntake intake;
  private double startPos;

  public HoldPiece(ClawIntake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    startPos = intake.getPosition();
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  @Override
  public void execute() {
    intake.setPosition(startPos);
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
  public void end(boolean interrupted) {}

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
