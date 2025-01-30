package frc.robot.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveLock extends Command{

    SwerveSubsystem swerve;

    public SwerveLock(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        swerve.lock();
    }

}
