package org.livoniawarriors.motorcontrol;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;

public class SparkMaxMotor extends SparkBaseMotor {

  public SparkMaxMotor(String motorName, int id) {
    this(motorName, new SparkMax(id, MotorType.kBrushless), false);
  }

  public SparkMaxMotor(String motorName, int id, MotorType type) {
    this(motorName, new SparkMax(id, type), false);
  }

  public SparkMaxMotor(String motorName, SparkMax motor, boolean logOnly) {
    super(motorName, motor, logOnly, DCMotor.getNEO(0));
  }

  public SparkMaxMotor(String motorName, SparkMax motor, boolean logOnly, DCMotor dcMotor) {
    super(motorName, motor, logOnly, dcMotor);
  }
}
