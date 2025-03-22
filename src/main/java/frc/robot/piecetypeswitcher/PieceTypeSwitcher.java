package frc.robot.piecetypeswitcher;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public class PieceTypeSwitcher extends SubsystemBase {

  @AutoLogOutput private String pieceSelected;

  public PieceTypeSwitcher() {
    super();
    pieceSelected = "Coral";
    AutoLogOutputManager.addObject(this);
  }

  public String getPieceSelected() {
    return pieceSelected;
  }

  public void togglePieceSelected() {
    if (pieceSelected == "Coral") {
      pieceSelected = "Algae";
    } else {
      pieceSelected = "Coral";
    }
  }

  public Command switchPieceSelected() {
    return runOnce(this::togglePieceSelected);
  }

  public boolean isCoral() {
    return pieceSelected == "Coral";
  }

  public boolean isAlgae() {
    return pieceSelected == "Algae";
  }

  public Color getPieceColor() {
    if (isCoral()) {
      return Color.kYellow;
    }
    return Color.kAqua;
  }
}
