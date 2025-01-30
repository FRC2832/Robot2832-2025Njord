package frc.robot.ramp;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampSubsystem extends SubsystemBase {
    VictorSPX motor;
    DigitalInput sensor;
    boolean pieceSeen;

    public RampSubsystem() {
        motor = new VictorSPX(5);
        sensor = new DigitalInput(0);
        motor.setInverted(true);
    }

    @Override
    public void periodic() {
        pieceSeen = !sensor.get();
        SmartDashboard.putBoolean("Sensor", pieceSeen);
    }

    public Command runMotor(double percent) {
        return run(() -> motor.set(ControlMode.PercentOutput, percent));
    }

    public Command runMotor(DoubleSupplier percent) {
        return run(() -> motor.set(ControlMode.PercentOutput, percent.getAsDouble()));
    }
}
