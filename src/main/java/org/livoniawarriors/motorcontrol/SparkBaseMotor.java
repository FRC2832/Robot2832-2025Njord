package org.livoniawarriors.motorcontrol;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

// TODO fix status code checking
@SuppressWarnings("unused")
class SparkBaseMotor extends MotorControl {
  private SparkBase motor;
  private SparkBaseConfig cfg;
  private RelativeEncoder encoder;
  private SparkClosedLoopController pid;

  private double scaleFactor;

  public SparkBaseMotor(String motorName, SparkBase motor, boolean logOnly, DCMotor dcMotor) {
    // need to persist PID as there is no way to get the PID values from motor
    super(motorName, logOnly, true);
    this.motor = motor;

    // don't retry config errors to boot faster
    motor.setCANMaxRetries(0);
    cfg = new SparkFlexConfig();
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    cfg.signals
        // unused frames
        .absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .iAccumulationAlwaysOn(false)
        // used frames
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(18)
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(18);

    cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Changes the measurement period and number of samples used to calculate the velocity for the
    // intergrated motor controller
    // Notability this changes the returned velocity and the velocity used for the onboard
    // velocity PID loop (triple check the PID portion of this statement)
    // Default settings of 32ms and 8 taps introduce ~100ms of measurement lag
    // https://www.chiefdelphi.com/t/shooter-encoder/400211/11
    // This value was taken from:
    // https://github.com/Mechanical-Advantage/RobotCode2023/blob/9884d13b2220b76d430e82248fd837adbc4a10bc/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/ModuleIOSparkMax.java#L132-L133
    // and tested on 9176 for YAGSL, notably 3005 uses 16ms instead of 10 but 10 is more common
    // based on github searches
    cfg.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);

    // set RPM pid
    final ClosedLoopSlot slot = ClosedLoopSlot.kSlot1;
    cfg.closedLoop.p(0.0000001, slot);
    cfg.closedLoop.i(0.0000001, slot);
    cfg.closedLoop.d(0, slot);
    cfg.closedLoop.velocityFF(
        12 / (10 * Units.radiansPerSecondToRotationsPerMinute(dcMotor.freeSpeedRadPerSec)), slot);
    cfg.closedLoop.iZone(0, slot);
    cfg.closedLoop.maxMotion.allowedClosedLoopError(0, slot);
    cfg.closedLoop.maxMotion.maxVelocity(0, slot);
    cfg.closedLoop.maxMotion.maxAcceleration(0, slot);
    // missing: kS, kG, kA

    var statusCode =
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // load data from NT, as we can't get them from motor API
    pidConstants.loadFromNT();
    configurePid();
  }

  @Override
  public void setBrakeMode(boolean brakeOn) {
    cfg.idleMode(brakeOn ? IdleMode.kBrake : IdleMode.kCoast);
    var statusCode =
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setPower(double percent) {
    motor.set(percent);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stopMotor(boolean coast) {
    motor.stopMotor();
  }

  @Override
  public void setScaleFactor(double factor) {
    // factor is inverted and usually less than 1, inverting so a 10:1 gearbox has a factor of 10
    scaleFactor = 1 / factor;
    cfg.encoder.positionConversionFactor(scaleFactor).velocityConversionFactor(scaleFactor);
    var statusCode =
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setInverted(boolean isInverted) {
    cfg.inverted(isInverted);
    var statusCode =
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public String[] getFaultArray() {
    return handleFaults(motor.getFaults(), motor.getWarnings());
  }

  @Override
  public String[] getStickyFaultArray() {
    return handleFaults(motor.getStickyFaults(), motor.getStickyWarnings());
  }

  private String[] handleFaults(Faults faults, Warnings warnings) {
    ArrayList<String> faultArray = new ArrayList<String>();

    if (faults.can) {
      faultArray.add("CAN");
    }
    if (faults.escEeprom) {
      faultArray.add("EEPROM");
    }
    if (faults.firmware) {
      faultArray.add("Firmware");
    }
    if (faults.gateDriver) {
      faultArray.add("GateDriver");
    }
    if (faults.motorType) {
      faultArray.add("MotorType");
    }
    if (faults.other) {
      faultArray.add("Other");
    }
    if (faults.sensor) {
      faultArray.add("Sensor");
    }
    if (faults.temperature) {
      faultArray.add("Temperature");
    }

    if (warnings.brownout) {
      faultArray.add("Brownout");
    }
    if (warnings.escEeprom) {
      faultArray.add("EEPROM_Warn");
    }
    if (warnings.extEeprom) {
      faultArray.add("ExtEEPROM_Warn");
    }
    if (warnings.hasReset) {
      faultArray.add("HasReset");
    }
    if (warnings.other) {
      faultArray.add("OtherWarn");
    }
    if (warnings.overcurrent) {
      faultArray.add("OverCurrent");
    }
    if (warnings.sensor) {
      faultArray.add("Sensor");
    }
    if (warnings.stall) {
      faultArray.add("Stall");
    }

    return faultArray.toArray(new String[0]);
  }

  @Override
  public String getVersion() {
    return motor.getFirmwareString();
  }

  @Override
  public double getSupplyVoltage() {
    return motor.getBusVoltage();
  }

  @Override
  public double getSupplyCurrent() {
    return motor.getOutputCurrent();
  }

  @Override
  public double getMotorVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public double getMotorPosition() {
    return encoder.getPosition() / scaleFactor;
  }

  @Override
  public double getMotorVelocity() {
    return encoder.getVelocity() / scaleFactor;
  }

  @Override
  public double getRpm() {
    return encoder.getVelocity() / scaleFactor;
  }

  @Override
  public void setCurrentLimit(double limit) {
    cfg.smartCurrentLimit((int) limit);
    var statusCode =
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setRpm(double rpm) {
    pid.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void setVelocity(double velocity) {
    pid.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void setPosition(double position) {
    pid.setReference(position, ControlType.kPosition);
  }

  @Override
  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public double getMotorTemp() {
    return motor.getMotorTemperature();
  }

  @Override
  public void clearStickyFaults() {
    var statusCode = motor.clearFaults();
  }

  @Override
  public boolean isConnected() {
    SmartDashboard.putString("Rev Status", motor.getLastError().name());
    var status = motor.getLastError();
    return status != REVLibError.kTimeout && status != REVLibError.kHALError;
  }

  @Override
  public boolean isFaulted() {
    return (motor.getFaults().rawBits & 0xFF) > 0;
  }

  /** Returns of type SparkFlex */
  @Override
  public Object getBaseMotor() {
    return motor;
  }

  @Override
  public void configurePid() {
    final ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;

    cfg.closedLoop.p(pidConstants.kP, slot);
    cfg.closedLoop.i(pidConstants.kI, slot);
    cfg.closedLoop.d(pidConstants.kD, slot);
    cfg.closedLoop.velocityFF(pidConstants.kV, slot);
    cfg.closedLoop.iZone(pidConstants.kiZone, slot);
    // cfg.closedLoop.maxMotion.allowedClosedLoopError(pidConstants.kiError, slot);
    // cfg.closedLoop.maxMotion.maxVelocity(pidConstants.kVelMax, slot);
    // cfg.closedLoop.maxMotion.maxAcceleration(pidConstants.kAccelMax, slot);
    // missing: kS, kG, kA

    var statusCode =
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    statusCode = statusCode;
  }

  @Override
  public void setSoftLimits(double back, double forward) {
    cfg.softLimit.forwardSoftLimit(forward);
    cfg.softLimit.forwardSoftLimitEnabled(true);
    cfg.softLimit.reverseSoftLimit(back);
    cfg.softLimit.reverseSoftLimitEnabled(true);
    var statusCode =
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
