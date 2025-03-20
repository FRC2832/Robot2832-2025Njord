package org.livoniawarriors.motorcontrol;

import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

abstract class MotorControl implements IMotorControl {
  protected NetworkTable motorTable;
  protected String name;
  protected boolean atSetpoint = false;
  protected boolean allowConfig;

  private DoublePublisher supplyVoltagePub;
  private DoublePublisher supplyCurrentPub;
  private DoublePublisher motorVoltagePub;
  private DoublePublisher posPub;
  private DoublePublisher veloPub;
  private DoublePublisher rawPosPub;
  private DoublePublisher rawVeloPub;
  private DoublePublisher rpmPub;
  private DoublePublisher tempPub;

  private BooleanPublisher setpointPub;
  private BooleanPublisher connectPub;
  private BooleanPublisher faultPub;

  private StringPublisher faultsPub;
  private StringPublisher stickyPub;
  private StringPublisher versionPub;

  public final PidConstants pidConstants;

  public MotorControl(String name, boolean logOnly, boolean persistPid) {
    this.name = name;
    allowConfig = !logOnly;

    // we don't persist this PID table because the motor remembers the value
    pidConstants = new PidConstants(name, persistPid);
    pidConstants.onChange(this::configurePid);

    // add motor to the list
    MotorControls.add(this);
    motorTable = NetworkTableInstance.getDefault().getTable("/Motors/" + name);

    supplyVoltagePub = motorTable.getDoubleTopic("Supply Voltage").publish();
    supplyCurrentPub = motorTable.getDoubleTopic("Supply Current").publish();
    motorVoltagePub = motorTable.getDoubleTopic("Motor Voltage").publish();
    posPub = motorTable.getDoubleTopic("Position").publish();
    veloPub = motorTable.getDoubleTopic("Velocity").publish();
    rawPosPub = motorTable.getDoubleTopic("Raw Position").publish();
    rawVeloPub = motorTable.getDoubleTopic("Raw Velocity").publish();
    rpmPub = motorTable.getDoubleTopic("RPM").publish();
    tempPub = motorTable.getDoubleTopic("Temperature").publish();

    setpointPub = motorTable.getBooleanTopic("At Setpoint").publish();
    connectPub = motorTable.getBooleanTopic("Connected").publish();
    faultPub = motorTable.getBooleanTopic("Faulted").publish();

    faultsPub = motorTable.getStringTopic("Faults").publish();
    stickyPub = motorTable.getStringTopic("Sticky Faults").publish();
    versionPub = motorTable.getStringTopic("Version").publish();

    //AdvantageKit logging
    Logger.recordOutput("/Motors/" + name + "/Supply Current", () -> logValues.supplyCurrent);
    Logger.recordOutput("/Motors/" + name + "/Supply Voltage", () -> logValues.supplyVoltage);
    Logger.recordOutput("/Motors/" + name + "/Motor Voltage", () -> logValues.motorVoltage);
    Logger.recordOutput("/Motors/" + name + "/Position", () -> logValues.position);
    Logger.recordOutput("/Motors/" + name + "/Velocity", () -> logValues.velocity);
    Logger.recordOutput("/Motors/" + name + "/Raw Position",  () -> logValues.motorPosition);
    Logger.recordOutput("/Motors/" + name + "/Raw Velocity", () -> logValues.motorVelocity);
    Logger.recordOutput("/Motors/" + name + "/RPM", () -> logValues.rpm);
    Logger.recordOutput("/Motors/" + name + "/Temperature", () -> logValues.motorTemp);
    Logger.recordOutput("/Motors/" + name + "/At Setpoint", () -> logValues.atSetpoint);
    Logger.recordOutput("/Motors/" + name + "/Connected", () -> logValues.isConnected);
    Logger.recordOutput("/Motors/" + name + "/Faulted", () -> logValues.isFaulted);
    Logger.recordOutput("/Motors/" + name + "/Faults", logValues.faults);
    Logger.recordOutput("/Motors/" + name + "/Sticky Faults", logValues.stickyFaults);
    Logger.recordOutput("/Motors/" + name + "/Version", logValues.version);
  }
  
  MotorLogValues logValues = new MotorLogValues();

  //update the values for advantagekit logging, for logging intermittently
  public void updateLogValues(){
    logValues.supplyCurrent = getSupplyCurrent();
    logValues.supplyVoltage = getSupplyVoltage();
    logValues.motorVoltage = getMotorVoltage();
    logValues.position = getPosition();
    logValues.velocity = getVelocity();
    logValues.motorPosition = getMotorPosition();
    logValues.motorVelocity = getMotorVelocity();
    logValues.rpm = getRpm();
    logValues.motorTemp = getMotorTemp();
    logValues.atSetpoint = atSetpoint();
    logValues.isConnected = isConnected();
    logValues.isFaulted = isFaulted();
    logValues.faults = getFaults();
    logValues.stickyFaults = getStickyFaults();
    logValues.version = getVersion();
  }

  protected void customLogging() {}

  public void updateLog() {
    setpointPub.set(atSetpoint());
    connectPub.set(isConnected());
    faultPub.set(isFaulted());

    faultsPub.set(getFaults());
    stickyPub.set(getStickyFaults());
    versionPub.set(getVersion());

    supplyVoltagePub.set(getSupplyVoltage());
    supplyCurrentPub.set(getSupplyCurrent());
    motorVoltagePub.set(getMotorVoltage());
    posPub.set(getPosition());
    veloPub.set(getVelocity());
    rawPosPub.set(getMotorPosition());
    rawVeloPub.set(getMotorVelocity());
    rpmPub.set(getRpm());
    tempPub.set(getMotorTemp());
    //advantagekit logging
    updateLogValues();
    customLogging();
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public boolean isFaulted() {
    return getFaultArray().length > 0;
  }

  @Override
  public String getFaults() {
    return String.join(" ", getFaultArray());
  }

  @Override
  public String getStickyFaults() {
    return String.join(" ", getStickyFaultArray());
  }

  @Override
  public boolean atSetpoint() {
    return atSetpoint;
  }
}
