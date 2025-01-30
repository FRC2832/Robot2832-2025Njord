package frc.robot.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class StraightDriveToPose extends Command {
    SwerveSubsystem swerve;
    Pose2d targetPose;
    double errorDistance;
    int finishedCounts;

    public StraightDriveToPose(SwerveSubsystem swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        errorDistance = 0;
        finishedCounts = 0;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var currentPose = swerve.getPose();
        var xDist = targetPose.getX() - currentPose.getX();
        var yDist = targetPose.getY() - currentPose.getY();
        //right now, we are using the distance as our speed in m/s.  Probably needs to be tuned...
        var targetSpeed = new Translation2d(xDist, yDist).times(15);
        swerve.drive(targetSpeed, 0, true);
        errorDistance = Math.pow((xDist * xDist) + (yDist * yDist), 0.5);

        if (errorDistance < Units.inchesToMeters(1)) {
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