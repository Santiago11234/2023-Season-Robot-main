package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase{
    private final PWMSparkMax neo;

    public WristSubsystem(){
       neo = new PWMSparkMax(Constants.IDs.WRIST_MOTOR);
       
    }

    public void setSpeed(double speed){
      neo.set(speed);
    }

    public void stop(){
        neo.set(0);
    }
}
//hi