package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldElevator extends Command {
  private Elevator elevator;
  private double startPos;

  public HoldElevator(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    startPos = elevator.getPosition();
  }

  @Override
  public void execute() {
    elevator.setPosition(startPos);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
