package org.livoniawarriors;

import edu.wpi.first.networktables.DoublePublisher;
import org.littletonrobotics.conduit.ConduitApi;

/**
 * This class asks AdvantageKit what the PDP currents are and puts them in a NT table with the
 * channel names, not numbers. Channels are named so that we don't need a dictionary when knowing
 * what channel his what device. Using AK's conduit API so that we don't have to requery the CAN
 * device.
 */
public class PdpLoggerKit implements Runnable {
  DoublePublisher[] channels;

  public PdpLoggerKit(String[] channelNames) {
    // can't access conduit yet as it might not have initialized yet
    channels = new DoublePublisher[channelNames.length];
    for (int i = 0; i < channelNames.length; i++) {
      channels[i] = UtilFunctions.getNtPub("/Pdp Channels/" + channelNames[i], 0.);
    }
  }

  @Override
  public void run() {
    ConduitApi conduit = ConduitApi.getInstance();
    for (int i = 0; i < conduit.getPDPChannelCount(); i++) {
      channels[i].set(conduit.getPDPChannelCurrent(i));
    }
  }
}
