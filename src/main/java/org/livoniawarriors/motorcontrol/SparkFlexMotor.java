package org.livoniawarriors.motorcontrol;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SparkFlexMotor extends SparkBaseMotor {
  public SparkFlexMotor(String motorName, int id) {
    this(motorName, new SparkFlex(id, MotorType.kBrushless), false);
  }

  public SparkFlexMotor(String motorName, int id, MotorType type) {
    this(motorName, new SparkFlex(id, type), false);
  }

  public SparkFlexMotor(String motorName, SparkFlex motor, boolean logOnly) {
    super(motorName, motor, logOnly, DCMotor.getNeoVortex(0));
  }

  public SparkFlexMotor(String motorName, SparkFlex motor, boolean logOnly, DCMotor dcMotor) {
    super(motorName, motor, logOnly, dcMotor);
  }
}
