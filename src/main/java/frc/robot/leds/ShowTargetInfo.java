package frc.robot.leds;

import org.livoniawarriors.leds.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.CameraType;
import frc.robot.vision.ColorMatchResult;

public class ShowTargetInfo extends Command {
    ILedSubsystem leds;
    AprilTagCamera camera;
    Color toShow;
    CameraType startingType;

    public ShowTargetInfo(ILedSubsystem leds, AprilTagCamera camera, Color toShow) {
        this.leds = leds;
        this.camera = camera;
        this.toShow = toShow;
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        startingType = camera.getPipeLineType();
        camera.setPipeLine(CameraType.ALGAE);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void execute() {
        var results = camera.getColorTargets();
        double length = leds.getLength();

        //start the leds as black
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(leds.getLength());
        for(int i=0; i<buffer.getLength(); i++) {
            buffer.setLED(i,Color.kBlack);
        }

        //if no targets were seen, just show black and leave
        if(results.size() == 0) {
            leds.setData(buffer);
            return;
        }
        
        //find result with biggest width of object seen
        ColorMatchResult colorMatch = new ColorMatchResult();
        for (ColorMatchResult colorMatchResult : results) {
            if (colorMatchResult.width > colorMatch.width) {
                colorMatch = colorMatchResult;
            }
        }
        
        //calculate where the color box boundaries are
        double midPoint = length / 2;
        double center = midPoint + (midPoint * colorMatch.centerX);
        double left = center - ((length * colorMatch.width) / 2);
        double right = center + ((length * colorMatch.width) / 2);

        //check to see if we go out of bounds
        int l = (int)Math.floor(left);
        if (l < 0) l=0; 
        int r = (int)Math.ceil(right);
        if (r > length - 1) r = (int)(length - 1);

        //show the box
        for(int i = l; i <= r; i++) {
            buffer.setLED(i,toShow);
        }
        leds.setData(buffer);
    }

    @Override
    public void end(boolean interrupted) {
        camera.setPipeLine(startingType);
    }
}
