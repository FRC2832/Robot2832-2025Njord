package frc.robot.piecetypeswitcher;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PieceTypeSwitcher extends SubsystemBase{
    
    @AutoLogOutput
    private String pieceSelected;

    public PieceTypeSwitcher(){
        pieceSelected = "Coral";
        AutoLogOutputManager.addObject(this);
    }

    public String getPieceSelected(){
        return pieceSelected;
    }
    
    public void setPieceSelected(){
        if(pieceSelected == "Coral"){
            pieceSelected = "Algae"; 
        }
        else{
            pieceSelected = "Coral";
        }
    }

    public Command switchPieceSelected(){
        return runOnce(this::setPieceSelected);
    }
}
