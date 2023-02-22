package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Constants;
import frc.robot.OI;

public class WristCommand extends CommandBase{
    private final WristSubsystem wrist;
    private final OI oi;
    public WristCommand(WristSubsystem wrist, OI oi){
        this.wrist = wrist;
        this.oi = oi;
    }
    @Override
    public void initialize(){
        wrist.stop();
    }
    @Override
    public void execute(){
        double speed = oi.getAxis(0, Constants.Axes.RIGHT_STICK_Y) * .05;
        wrist.setSpeed(speed);
    }
    @Override
    public void end(boolean interrupted){
        wrist.stop();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
