package org.livoniawarriors.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class SolidColorLeds extends Command {
  ILedSubsystem leds;
  AddressableLEDBuffer m_ledBuffer;

  public SolidColorLeds(ILedSubsystem leds, Color color) {
    this.leds = leds;
    addRequirements(leds);
    m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initialize() {
    leds.setData(m_ledBuffer);
  }

  @Override
  public void execute() {
    // since we set the color in init, no need to repeat it
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
