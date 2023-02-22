package frc.robot.commands;
//if angle is not flat go "uphill"
//if angle is flat for 3 seconds, isFinisehd returns true

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//to move forward/backwards do: swerve.setAllState(new SwerveModuleState(speed, new Rotation2d()))
//getPitch() - angle of the front of the robot off the ground 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.swerve.SwerveMovement;
import frc.robot.swerve.SwerveUtil;
import frc.robot.math.vec2;
import frc.robot.subsystems.NavX;

public class AutoBalanceCommand extends CommandBase{
    private final SwerveDriveSubsystem swerve;
    private final NavX navx;
    public AutoBalanceCommand(SwerveDriveSubsystem swerve, NavX navx){
        this.swerve = swerve;
        this.navx = navx;
        addRequirements(swerve);
        Timer.getFPGATimestamp();
        

    }
    public void initialize(){

    }
    public void execute(){

        if(!navx.isZero()){
            double xSpeed = Math.max(Math.min(navx.getPitch()/23, 1), -1);
            double ySpeed = Math.max(Math.min(navx.getRoll()/23, 1), -1);
            
            SwerveMovement movement = new SwerveMovement(new vec2(-xSpeed, -ySpeed), 0);

            swerve.setMovement(movement);
        }   
        else{
            swerve.setModules(new SwerveModuleState());
            Timer.getFPGATimestamp();

        }
        

     }
    public void end(){

    }

    public boolean isFinished(){
        return false;
    }
}
