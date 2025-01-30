package frc.robot.leds;

import org.livoniawarriors.leds.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class FrontLeds implements ILedSubsystem {
    static final int NUM_LEDS = 27;
    AddressableLED leds;
    AddressableLEDBuffer fullBuffer;
    AddressableLEDBuffer frontBuffer;

    /**
     * Setup the Led Hardware
     * @param channel What PWM channel the strip is connected to
     * @param length How long the LED string is (full robot length)
     */
    public FrontLeds(int channel, int length) {
        super();
        //create the hardware device on the specified
        leds = new AddressableLED(channel);
        fullBuffer = new AddressableLEDBuffer(length);
        frontBuffer = new AddressableLEDBuffer(NUM_LEDS);
        leds.setLength(length);
        //start the strip
        leds.start();
    }

    @Override
    public void periodic() {
        //actually command the leds to show the pattern
        leds.setData(fullBuffer);
    }
    
    @Override
    public int getLength() {
        return NUM_LEDS;
    }

    @Override
    public Color getLed(int index) {
        return frontBuffer.getLED(index);
    }

    @Override
    public void setLed(int index, Color color) {
        frontBuffer.setLED(index, color);
        fullBuffer.setLED(index, color);
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        frontBuffer = buffer;
        //copy data into shown buffer
        for(int i=0; i<NUM_LEDS; i++) {
            fullBuffer.setLED(i, frontBuffer.getLED(i));
        }
    }

    public void setRearLed(int index, Color color) {
        fullBuffer.setLED(index + NUM_LEDS, color);
    }

    public void setRearData(AddressableLEDBuffer buffer) {
        //copy data into shown buffer
        for(int i=0; i<buffer.getLength(); i++) {
            fullBuffer.setLED(i + NUM_LEDS, buffer.getLED(i));
        }
    }
}
