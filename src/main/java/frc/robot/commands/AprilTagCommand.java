package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.network.vision.LimeLight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.swerve.SwerveMovement;
import frc.robot.swerve.SwerveUtil;

public class AprilTagCommand extends CommandBase{
    private SwerveDriveSubsystem swerve;
    private NavX navx;
    public void initialize(SwerveDriveSubsystem swerveDriveSubsystem, NavX navx) {
        this.swerve = swerveDriveSubsystem;
        this.navx = navx;
        addRequirements(swerveDriveSubsystem);
    }
    public void execute() {
        if(LimeLight.getZDistance() != -1) {
            SwerveMovement m = SwerveUtil.toRobotCentric(new SwerveMovement(LimeLight.getZDistance(), LimeLight.getXDistance(), LimeLight.getXRotation()), navx.getHeading());
            swerve.setMovement(m);
             
        }
    }
    public void end() {

    }
    public boolean isFinished() {
        return false;
    }
}
