package frc.robot.clawpivot;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldClawPivot extends Command {
  private ClawPivot clawpivot;
  private double startPos;

  public HoldClawPivot(ClawPivot clawpivot) {
    this.clawpivot = clawpivot;
    addRequirements(clawpivot);
  }

  @Override
  public void initialize() {
    startPos = clawpivot.getAngle();
  }

  @Override
  public void execute() {
    clawpivot.setAngle(startPos);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
