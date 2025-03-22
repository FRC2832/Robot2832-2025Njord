package frc.robot.clawintake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public class ClawIntakeSim extends ClawIntake {
  FlywheelSim sim;
  double power;
  @AutoLogOutput double linearSpeed_mps;
  double shot_speed_mps;

  // TODO change to 12:56
  // The intake has 12:30 belt pullys, then 1:1 throughout the rest, wheels are 2" OD
  // estimation from recalc
  // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&flywheelMomentOfInertia=%7B%22s%22%3A0%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A0%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%2A%22%7D&motorRatio=%7B%22magnitude%22%3A2.5%2C%22ratioType%22%3A%22Reduction%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A1.2%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A4.5%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A3%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A900%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0
  public ClawIntakeSim() {
    super();
    sim =
        new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.65, 0.1), DCMotor.getKrakenX60(1));
    power = 0;
    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void simulationPeriodic() {
    sim.setInput(power * RobotController.getBatteryVoltage());
    sim.update(Robot.kDefaultPeriod);
    linearSpeed_mps =
        sim.getAngularVelocity().in(RevolutionsPerSecond) * Math.PI * Inches.of(2).in(Meter);
    // estimation from ReCalc
    shot_speed_mps = linearSpeed_mps / 6;
  }

  @Override
  void setPower(double power) {
    this.power = power;
  }

  @Override
  public boolean hasCoral() {
    return true;
  }

  @Override
  public boolean hasAlgae() {
    return false;
  }

  @Override
  void setVelocity(double velocity) {}

  @Override
  void setPosition(double position) {}

  @Override
  double getPosition() {
    return 0;
  }

  @Override
  void updateSensors() {}

  @Override
  double getVelocity() {
    return 0;
  }

  @Override
  void setRpm(double rpm) {}
}
