
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.DriveCommand;
//import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.TelescopeCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ObstructionSensor;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.network.vision.LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    private final OI oi = new OI();
    private final NavX navx = new NavX();

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(navx);
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ObstructionSensor clawObstructionSensor = new ObstructionSensor(0);
    private final ArmPivotSubsystem armPivotSubsystem = new ArmPivotSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
       
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final Trigger clawObstructedTrigger;
    private double lastClawOpenTime = Double.NEGATIVE_INFINITY;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // swerveDriveSubsystem.setDefaultCommand(new DriveCommand(swerveDriveSubsystem, oi, navx));
        // armPivotSubsystem.setDefaultCommand(new ArmCommand(armPivotSubsystem, oi));
        //elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, oi));

    //    telescopeSubsystem.setDefaultCommand(new TelescopeCommand(telescopeSubsystem, oi));
        // wristSubsystem.setDefaultCommand(new WristCommand(wristSubsystem, oi));
        swerveDriveSubsystem.setDefaultCommand(new DriveCommand(swerveDriveSubsystem, oi, navx));

      oi.getButton(0, Constants.Buttons.Y_BUTTON).onTrue(new RunCommand(()->{
        elevatorSubsystem.setPosition(.1);
      }, elevatorSubsystem));


      oi.getButton(0, Constants.Buttons.X_BUTTON).onTrue(new RunCommand(()->{
        elevatorSubsystem.setPosition(.2);
      }, elevatorSubsystem));

      oi.getButton(0, Constants.Buttons.A_BUTTON).onTrue(new RunCommand(()->{
        elevatorSubsystem.setPosition(.4);
      }, elevatorSubsystem));


        clawObstructedTrigger = new Trigger(() ->
            clawObstructionSensor.isObstructed()
            && Timer.getFPGATimestamp() > lastClawOpenTime + Constants.Timing.CLAW_DELAY_AFTER_OPEN
        );

        // configureButtonBindings();

        //oi.getButton(1, Constants.Buttons.Y_BUTTON).onTrue(new ElevatorCommand(elevatorSubsystem, oi));

      //  wrist.setDefaultCommand(new WristCommand(wrist, oi));
    }

    /*
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
        
    private void configureButtonBindings() {
        clawObstructedTrigger.onTrue(new CloseClawCommand(clawSubsystem).andThen(() -> {
            lastClawOpenTime = Double.POSITIVE_INFINITY;
        }, clawSubsystem));
        
        oi.getButton(0, Constants.Buttons.Y_BUTTON).onTrue(
            new RunCommand(
                () -> {
                    elevatorSubsystem.setPosition(0);
                }, 
                elevatorSubsystem
            ));
        oi.getButton(0, Constants.Buttons.B_BUTTON).onTrue(
            new OpenClawCommand(clawSubsystem)
            .andThen(() ->
                lastClawOpenTime = Timer.getFPGATimestamp(),
                clawSubsystem
            )
        );

        oi.getButton(0, Constants.Buttons.X_BUTTON).onTrue(
            new AutoBalanceCommand(swerveDriveSubsystem, navx)
        );
        
        oi.getButton(0, Constants.Buttons.A_BUTTON).onTrue(
            new CloseClawCommand(clawSubsystem)
        );

        oi.getButton(0, Constants.Buttons.Y_BUTTON).whileTrue(new CommandBase(){
            {
                addRequirements(swerveDriveSubsystem);
            }

           // @Override
            // public void execute(){
            //     swerveDriveSubsystem.setModules(new SwerveModuleState(
            //         1,
            //         new Rotation2d()
            //     ));
            // }
        });
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.RobotInfo.Auton.kMaxSpeedMetersPerSecond,
            Constants.RobotInfo.Auton.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(
            Constants.RobotInfo.DriveConstants.kDriveKinematics
        );

        //sample trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);
        
         // 3. Define PID controllers for tracking trajectory
         PIDController xController = new PIDController(Constants.RobotInfo.Auton.kPXController, 0, 0);
         PIDController yController = new PIDController(Constants.RobotInfo.Auton.kPYController, 0, 0);
         ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.RobotInfo.Auton.kPThetaController,
             0, 0,
             Constants.RobotInfo.Auton.kThetaControllerConstraints
         );
         thetaController.enableContinuousInput(-Math.PI, Math.PI);

         SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveDriveSubsystem::getPose,
                Constants.RobotInfo.DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveDriveSubsystem::getRotation,
                swerveDriveSubsystem::setModules,
                swerveDriveSubsystem);

         swerveDriveSubsystem.resetPose(trajectory.getInitialPose());           
         return swerveControllerCommand.andThen(
             new RunCommand(
                 swerveDriveSubsystem::stop,
                swerveDriveSubsystem
            )
         );

    }

    public void periodic(){
//        LimeLight.getOffsetFromCenteredAprilTag();
    }
}
