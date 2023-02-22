package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final PWMSparkMax armPivotMotor1;
    private final PWMSparkMax armPivotMotor2;

    // private final PIDController pid; 
    // private final CANCoder canCoder;

    public ArmPivotSubsystem(){
        // pid = new PIDController(0, 0, 0);
        armPivotMotor1 = new PWMSparkMax(Constants.IDs.ARM_PIVOT_1);
        armPivotMotor2 = new PWMSparkMax(Constants.IDs.ARM_PIVOT_2);
        // canCoder = new CANCoder(Constants.IDs.ENCODER_3);
    }

    public void stop(){
        armPivotMotor1.stopMotor();
        armPivotMotor2.stopMotor();
    }

    public void set(double speed){
        armPivotMotor1.set(speed);
        armPivotMotor2.set(-speed);
    }

    // public void setAngle(double angle){
    //     pid.setSetpoint(angle);
    //     pid.setSetpoint(angle);
    // }

    @Override
    public void periodic( ) {
    //    double speed =  pid.calculate(canCoder.getPosition());
    //    set(speed);
    }
}