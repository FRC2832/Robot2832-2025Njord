package frc.robot.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FinePosition extends SequentialCommandGroup {
    SwerveSubsystem swerve;
    Pose2d targetPose;

    public FinePosition(SwerveSubsystem swerve, Pose2d targetPose) {
        super(
            new TurnRobotToPose(swerve, targetPose),
            new TurnWheelsToPose(swerve, targetPose),
            new StraightDriveToPose(swerve, targetPose)
        );
    }
}
