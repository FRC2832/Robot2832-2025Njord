package frc.robot.swervedrive;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PushPartner extends Command {
    SwerveSubsystem swerve;
    Pose2d startPose;
    double speed;

    public PushPartner(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startPose = swerve.getPose();
        speed = 1;
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(-speed, 0),0,false);
        //ramp up speed if robots are tough to move (50 loops per second)
        speed += 0.05;
    }

    @Override
    public boolean isFinished() {
        return PhotonUtils.getDistanceToPose(startPose, swerve.getPose()) > Units.inchesToMeters(6);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
