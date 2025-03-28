package org.livoniawarriors.motorcontrol;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import java.util.ArrayList;

/** Support class for all the motors */
public final class MotorControls {
  private static ArrayList<MotorControl> motors = new ArrayList<>();
  private static ArrayList<FieldsToLog> globalLogs = new ArrayList<>();

  public static void add(MotorControl motor) {
    motors.add(motor);

    var inst = NetworkTableInstance.getDefault();
    var name = motor.getName();
    FieldsToLog logger = new FieldsToLog();
    logger.motorOutput = inst.getTable("Motor Outputs").getDoubleTopic(name).publish();
    logger.motorCurrent = inst.getTable("Motor Currents").getDoubleTopic(name).publish();
    logger.motorTemp = inst.getTable("Motor Temps").getDoubleTopic(name).publish();
    logger.faults = inst.getTable("Faults").getStringTopic(name).publish();
    logger.stickyFaults = inst.getTable("Sticky Faults").getStringTopic(name).publish();
    logger.connected = inst.getTable("Connected").getBooleanTopic(name).publish();
    globalLogs.add(logger);
  }

  public static void ClearStickyFaults() {
    for (var motor : motors) {
      motor.clearStickyFaults();
    }
  }

  static final int NUM_LOOPS_LOGGED = 3;
  static int Counts = 0;

  public static void UpdateLogs() {
    for (int i = 0; i < motors.size(); i++) {
      // too much data is blocking robot functions, so we are logging less frequently
      if ((Counts + i) % NUM_LOOPS_LOGGED != 0) {
        continue;
      }
      var motor = motors.get(i);
      var log = globalLogs.get(i);
      motor.updateLog();
      log.connected.set(motor.isConnected());
      log.faults.set(motor.getFaults());
      log.stickyFaults.set(motor.getStickyFaults());
      log.motorOutput.set(motor.getMotorVoltage());
      log.motorCurrent.set(motor.getSupplyCurrent());
      log.motorTemp.set(motor.getMotorTemp());
    }
    Counts++;
  }
}

class FieldsToLog {
  public DoublePublisher motorOutput;
  public DoublePublisher motorCurrent;
  public DoublePublisher motorTemp;
  public StringPublisher faults;
  public StringPublisher stickyFaults;
  public BooleanPublisher connected;
}
