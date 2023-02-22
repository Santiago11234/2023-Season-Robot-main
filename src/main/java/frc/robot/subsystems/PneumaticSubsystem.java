package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase{
    private final Solenoid m_solenoid;

    public PneumaticSubsystem(){
        m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    } 

    public void extendPiston() {
        m_solenoid.set(true);
    }
    
    public void RetractPiston() {
        m_solenoid.set(false);
    }
}
