package frc.robot.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnRobotToPose extends Command {
    SwerveSubsystem swerve;
    Pose2d targetPose;
    Rotation2d deltaAngle;
    int finishedCounts = 0;

    public TurnRobotToPose(SwerveSubsystem swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        deltaAngle = swerve.getPose().getRotation().minus(targetPose.getRotation());
        swerve.drive(new Translation2d(), -deltaAngle.getRadians() * 0.07, true);

        if (Math.abs(deltaAngle.getDegrees()) < 2) {
            finishedCounts++;
        } else {
            finishedCounts = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return finishedCounts >= 5;
    }
}
