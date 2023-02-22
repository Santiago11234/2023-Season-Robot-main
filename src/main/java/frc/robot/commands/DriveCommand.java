package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.swerve.SwerveMovement;
import frc.robot.swerve.SwerveUtil;

public class DriveCommand extends CommandBase {
    //private LimeLight light;
    private final OI oi;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final NavX navX;

    public DriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, OI oi, NavX navX) {
        this.oi = oi;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.navX = navX;

        addRequirements(swerveDriveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SwerveMovement swerveMovement;
        // if(LimeLight.getZDistance() != -1) {
        //     swerveMovement = SwerveUtil.toRobotCentric(new SwerveMovement(LimeLight.getZDistance(), LimeLight.getXDistance(), LimeLight.getXRotation()), navX.getHeading());
        // }
        // else {
        var rawMovement = getRawMovement();
        swerveMovement = SwerveUtil.toRobotCentric(rawMovement, navX.getHeading());
        SmartDashboard.putString("swerve movement", swerveMovement.toString());
        swerveDriveSubsystem.setMovement(swerveMovement);
    }

    public SwerveMovement getRawMovement(){
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double rawRot = oi.getAxis(0, Constants.Axes.RIGHT_STICK_X);
        
        return new SwerveMovement(rawY * .6, -rawX * .6, rawRot * .5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}