package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem  extends SubsystemBase{

    public WPI_VictorSPX motor;
    private final CANCoder encoder;
    private final PIDController pid;

    public TelescopeSubsystem() {
        motor = new WPI_VictorSPX(Constants.IDs.TELESCOPE_MOTOR);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        encoder = new CANCoder(Constants.IDs.TELESCOPE_ENCODER);
        pid = new PIDController(0.0000000001, 0, 0);
    }

    // public void setPosition(double position) {
    //     pid.setSetpoint(position);
    // }

    public void extend(double speed) {
        motor.set(speed);
    }

    // @Override
    // public void periodic() {
    //     double voltage = pid.calculate(encoder.getPosition());
    //     motor.set(voltage);
    // }
}
