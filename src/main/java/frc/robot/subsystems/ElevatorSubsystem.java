package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;

public class ElevatorSubsystem  extends SubsystemBase{
    private final WPI_TalonFX motorLeft;
    private final WPI_TalonFX motorRight;

    private final PIDController pid;

    private final double encoderOffset;

    public static final List<WPI_TalonFX> motors = new ArrayList<>();

    public ElevatorSubsystem() {
        motorLeft = new WPI_TalonFX(Constants.IDs.ELEVATOR_MOTOR_LEFT);
        motorRight = new WPI_TalonFX(Constants.IDs.ELEVATOR_MOTOR_RIGHT);
        motorLeft.configFactoryDefault();
        motorRight.configFactoryDefault();
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
        
        getEncoderMotor().configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        encoderOffset = getEncoderMotor().getSelectedSensorPosition();

        pid = new PIDController(3.0, 1.0, 0);

        motors.add(motorLeft);
        motors.add(motorRight);
    }

    private WPI_TalonFX getEncoderMotor(){
        return motorLeft;
    }

    private double getPosition() {
        double rawEncoderValue = getEncoderMotor().getSelectedSensorPosition();
        return (rawEncoderValue - encoderOffset) * .00001;
    }

    public void setPosition(double position) {
        pid.setSetpoint(position);
    }

    private void setSpeed(double speed) {
         motorLeft.set(speed);
         motorRight.set(-speed);
    }

    @Override
    public void periodic() {
        double rawVoltage = pid.calculate(getPosition());
        // Artificially cap the voltage
        double voltage = Math.signum(rawVoltage) * Math.min(Math.abs(rawVoltage), 0.15);
        SmartDashboard.putNumber("elevator position", getPosition());
        setSpeed(voltage);
//        stop();
        SmartDashboard.putNumber("elevator voltage", voltage);
        SmartDashboard.putNumber("elevator setpoint", pid.getSetpoint());
    }

    public void stop(){
        motorLeft.set(0);
        motorRight.set(0);
    }
}
