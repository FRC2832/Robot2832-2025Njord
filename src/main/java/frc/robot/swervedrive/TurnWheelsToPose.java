package frc.robot.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnWheelsToPose extends Command {
    SwerveSubsystem swerve;
    Pose2d targetPose;
    boolean finished;

    public TurnWheelsToPose(SwerveSubsystem swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        finished = false;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Rotation2d deltaAngle = swerve.getPose().getRotation().minus(targetPose.getRotation());
        var currentStates = swerve.getModules();
    
        finished = true;
        SwerveModuleState[] outputs = new SwerveModuleState[4];
        for (int i=0; i<4; i++) {
            //request an optimized angle for each wheel
            SwerveModuleState request = new SwerveModuleState(0, deltaAngle);
            request.optimize(currentStates[i].getState().angle);
            outputs[i] = request;

            //check if wheel is good enough to continue
            var errorAngle = Math.abs(deltaAngle.minus(request.angle).getDegrees());
            //handle if optimized request is 180* off
            if (errorAngle > 90) {
                errorAngle = Math.abs(errorAngle - 180);
            }

            if (errorAngle > 3) {
                finished = false;
            }
        }

        swerve.setModuleStates(outputs, true);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
