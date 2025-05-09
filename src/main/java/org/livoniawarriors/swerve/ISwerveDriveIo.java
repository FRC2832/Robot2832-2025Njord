package org.livoniawarriors.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveDriveIo {
  void updateInputs();

  Translation2d[] getCornerLocations();

  String[] getModuleNames();

  void setTurnMotorBrakeMode(boolean brakeOn);

  void setDriveMotorBrakeMode(boolean brakeOn);

  /**
   * Returns the CANcoder absolute angle of the swerve corner in degrees
   *
   * @param wheel Which corner to look at
   * @return The absolute angle of the swerve corner in degrees
   */
  double getCornerAbsAngle(int wheel);
  /**
   * Returns the turn motor absolute angle of the swerve corner in degrees
   *
   * @param wheel Which corner to look at
   * @return The absolute angle of the swerve corner in degrees
   */
  double getCornerAngle(int wheel);
  /**
   * Returns the speed of the swerve corner in meters per second
   *
   * @param wheel Which corner to look at
   * @return The speed of the swerve corner in meters per second
   */
  double getCornerSpeed(int wheel);

  /**
   * Returns the distance the swerve corner as traveled in meters
   *
   * @param wheel Which corner to look at
   * @return The distance the swerve corner as traveled in meters
   */
  double getCornerDistance(int wheel);

  /**
   * Sets the offset for the corner based on the configuration
   *
   * @param wheel
   * @param angle
   */
  void setCorrectedAngle(int wheel, double angle);

  void setCornerState(int wheel, SwerveModuleState swerveModuleState);

  void resetWheelPositions();

  double getDriveVoltage(int wheel);

  void setDriveVoltage(int wheel, double volts);
}
