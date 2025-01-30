package frc.robot.elevator;

import org.livoniawarriors.motorcontrol.TalonFXMotor;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;

public class ElevatorHw extends Elevator {
    TalonFXMotor leftMotor;
    TalonFXMotor rightMotor;
    LaserCan distSensor;

    public ElevatorHw() {
        super();

        leftMotor = new TalonFXMotor("Left Elevator", 8);
        rightMotor = new TalonFXMotor("Right Elevator", 9, "rio", true);
        leftMotor.setSoftLimits(16, 80);
        leftMotor.setScaleFactor(1);
        
        distSensor = new LaserCan(0);
        /*
        TalonFX leftKracken = (TalonFX)leftMotor.getBaseMotor();
        TalonFX rightKracken = (TalonFX)rightMotor.getBaseMotor();
        var follower = new Follower(leftKracken.getDeviceID(), true);
        rightKracken.setControl(follower);
        */
    }
    
    @Override
    public void setPosition(double distance) {
        if (pidEnabled) {
            leftMotor.setPosition(distance);
        }
    }

    @Override
    public void setPower(double pct) {
        leftMotor.setPower(pct);
    }

    @Override
    public double getPosition() {
        return leftMotor.getPosition();
    }

    @Override
    public double getDistanceSensor() {
        return distSensor.getMeasurement().distance_mm * 1000;
    }

    @Override
    public void setEncoderPosition(double position) {
        leftMotor.setEncoderPosition(position);
    }

}
