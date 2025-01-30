package frc.robot.leds;

import org.livoniawarriors.leds.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class RearLeds implements ILedSubsystem {
    static final int NUM_LEDS = 27;
    FrontLeds front;
    AddressableLEDBuffer buffer;

    public RearLeds(FrontLeds front) {
        super();
        this.front = front;
        buffer = new AddressableLEDBuffer(NUM_LEDS);
    }
    
    @Override
    public int getLength() {
        return NUM_LEDS;
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        this.buffer = buffer;

        //need to invert buffer to report it to the front strip
        int size = buffer.getLength();
        AddressableLEDBuffer frontReqBuffer = new AddressableLEDBuffer(size);
        for(int i=0; i<size; i++) {
            frontReqBuffer.setLED(NUM_LEDS - i - 1, buffer.getLED(i));
        }
        front.setRearData(frontReqBuffer);
    }

    @Override
    public Color getLed(int index) {
        return buffer.getLED(index);
    }

    @Override
    public void setLed(int index, Color color) {
        //invert request to appear left to right in the command
        front.setRearLed(NUM_LEDS - index - 1, color);
    }
    
}
