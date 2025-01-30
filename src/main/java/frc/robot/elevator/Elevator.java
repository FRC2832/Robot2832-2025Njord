package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {
    public abstract void setPosition(double distance);
    public abstract void setPower(double pct);
    public abstract double getPosition();
    public abstract double getDistanceSensor();
    public abstract void setEncoderPosition(double position);
    boolean pidEnabled;

    public Elevator() {
        super();
        pidEnabled = false;
    }

    @Override
    public void periodic() {

    }
}
