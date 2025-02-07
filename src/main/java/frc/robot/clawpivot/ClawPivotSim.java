package frc.robot.clawpivot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class ClawPivotSim extends ClawPivot {
  SingleJointedArmSim sim;
  double voltage;
  PIDController m_controller;
  ArmFeedforward m_feedforward;

  public ClawPivotSim() {
    super();

    //claw pivot has a gearbox with a 10:58 and 18:58 ratio gears, then a 12:30 chain, making a final ratio of 46.72:1
    //estimations from https://www.reca.lc/arm?armMass=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A10%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A180%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&ratio=%7B%22magnitude%22%3A46.72%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
    sim =
        new SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(0.89, 0.01),
            DCMotor.getKrakenX60(1),
            46.72,
            Inches.of(8).in(Meters),
            0,
            Math.toRadians(200),
            false,
            0);
    voltage = 0;
    m_controller = new PIDController(.1, 0.01, 0);
    m_feedforward = new ArmFeedforward(0, 0.07, 0.89, 0.01);
  }

  @Override
  public void simulationPeriodic() {
    sim.setInput(voltage);
    sim.update(Robot.kDefaultPeriod);
  }

  @Override
  void setPower(double power) {
    voltage = power * RobotController.getBatteryVoltage();
  }

  @Override
  void setAngle(double angle) {
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(getAngle(), angle);
    double feedforwardOutput = m_feedforward.calculate(getAngle(), 0);
    voltage = pidOutput + feedforwardOutput;
    voltage =
        Math.signum(voltage) * Math.min(Math.abs(voltage), RobotController.getBatteryVoltage());
  }

  @Override
  public double getSensorAngle() {
    return getAngle();
  }

  @Override
  public double getAngle() {
    return Math.toDegrees(sim.getAngleRads());
  }

  @Override
  void setEncoderPosition(double position) {
    sim.setState(Math.toRadians(position), sim.getVelocityRadPerSec());
  }
}
