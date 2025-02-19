package frc.robot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;

public class ElevatorSimul extends Elevator {
  ElevatorSim sim;
  double voltage;
  PIDController m_controller;
  ElevatorFeedforward m_feedforward;

  public ElevatorSimul() {
    // gearbox has 2 Krakens at a 12:60 gear ratio, then a 16:16 chain link, with a driving links at
    // a 22T pulley (Chain Spool OD 1.981")
    // initial height is measured from intake pivot point, should be ~16.5" off the ground, with a
    // max of 80"
    // https://wcproducts.info/files/frc/drawings/Web-%2325%20Double%20Hub%20Sprockets.pdf
    // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A24%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A20%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&ratio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.981%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A60%2C%22u%22%3A%22in%22%7D
    sim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            5,
            Pounds.of(20).in(Kilograms),
            Inches.of(1.981).in(Meters) / 2,
            Inches.of(16).in(Meters),
            Inches.of(80).in(Meters),
            false,
            Inches.of(16.5).in(Meters));
    m_controller = new PIDController(.1, 0.01, 0);
    m_feedforward = new ElevatorFeedforward(0, 0.38, 1.52, 0.04);
    voltage = 0;
  }

  @Override
  public void simulationPeriodic() {
    sim.setInput(voltage);
    sim.update(Robot.kDefaultPeriod);
  }

  @Override
  public void setPosition(double distance) {
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(getPosition(), distance);
    double feedforwardOutput = m_feedforward.calculate(sim.getVelocityMetersPerSecond());
    voltage = pidOutput + feedforwardOutput;
    voltage =
        Math.signum(voltage) * Math.min(Math.abs(voltage), RobotController.getBatteryVoltage());
  }

  @Override
  public void setPower(double pct) {
    voltage = pct * RobotController.getBatteryVoltage();
  }

  @Override
  public double getMotorPosition() {
    return Units.metersToInches(sim.getPositionMeters());
  }

  @Override
  public double getDistanceSensor() {
    return getMotorPosition();
  }

  @Override
  public void setEncoderPosition(double position) {
    sim.setState(Units.inchesToMeters(position), sim.getVelocityMetersPerSecond());
  }

  @Override
  void updateSensor() {}
}
