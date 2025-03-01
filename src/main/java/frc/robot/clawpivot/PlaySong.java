package frc.robot.clawpivot;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.Command;

public class PlaySong extends Command {
  private Orchestra music;

  public PlaySong(ClawPivot pivot) {
    addRequirements(pivot);

    music = new Orchestra();

    // Add a single device to the orchestra
    music.addInstrument(pivot.getMotor());

    // Attempt to load the chrp
    var status = music.loadMusic("output.chrp");

    if (!status.isOK()) {
      // log error
    }
  }

  @Override
  public void initialize() {
    music.play();
  }

  @Override
  public void end(boolean interrupted) {
    music.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
