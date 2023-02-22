// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
	public static final class IDs{
		
		// Swerve uses up motor ids 1-12
				public static final SwerveModuleConfig MODULE_FRONT_LEFT  = new SwerveModuleConfig(1, 2, 9 , 126.1231);//124.77
		public static final SwerveModuleConfig MODULE_FRONT_RIGHT = new SwerveModuleConfig(3, 4, 10,-124.1016);//233.877
		public static final SwerveModuleConfig MODULE_BACK_LEFT   = new SwerveModuleConfig(5, 6, 11, 10.6348);//9.668
		public static final SwerveModuleConfig MODULE_BACK_RIGHT  = new SwerveModuleConfig(7, 8, 12, 48.7793);//50.400

		public static final int ARM_PIVOT_1 = 13;
		public static final int ARM_PIVOT_2 = 14;
		public static final int ELEVATOR_MOTOR_LEFT = 15;
		public static final int ELEVATOR_MOTOR_RIGHT = 16;
		public static final int TELESCOPE_ENCODER = 20;
		public static final int ENCODER_3 = 20;
		public static final int CLAW_SOLENOID_CHANNEL = 3;	

		public static final int TELESCOPE_MOTOR = 20;
		public static final int WRIST_MOTOR = 21;	

	}

	public static final class RobotInfo {
		public static final double robotBaseLength = 0.44;

		public static final double SWERVE_KP = 0.1;
		public static final double SWERVE_KI = 0.1;

		public static final double MOVEMENT_SPEED = 0.3; // 0 - 1
		public static final double MAX_VELOCITY = 4;
		public static final double MAX_ACCELERATION = 3;

		//https://github.com/SeanSun6814/FRC0ToAutonomous/blob/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot/Constants.java#L57
		public static class DriveConstants {
			public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        	public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

			public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d( robotBaseLength,  robotBaseLength),//front left
            new Translation2d(  robotBaseLength, - robotBaseLength), //front right
            new Translation2d(- robotBaseLength,  robotBaseLength), //back left
            new Translation2d(-robotBaseLength,  -robotBaseLength) //back right
            //originally
            /* back left -front left- back right- front right */
        );
		}
		public static final double ROTATOR_MOTOR_KP = 0.005;
		public static final double ROTATOR_MOTOR_KI = 0;

		public static class Auton {
			public static final double POSITION_KP = 5.0;
			public static final double POSITION_KI = 0.0;

			public static final double ROTATION_KP = 5.0;
			public static final double ROTATION_KI = 0.0;

			public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
			public static final double kMaxAngularSpeedRadiansPerSecond = //
					DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
			public static final double kMaxAccelerationMetersPerSecondSquared = 3;
			public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
			public static final double kPXController = 1.5;
			public static final double kPYController = 1.5;
			public static final double kPThetaController = 3;
			
			public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
			new TrapezoidProfile.Constraints(
					kMaxAngularSpeedRadiansPerSecond,
					kMaxAngularAccelerationRadiansPerSecondSquared);
		}
	}

	public static final class Timing {
		public static final double CLAW_DELAY_AFTER_OPEN = 0.5;
	}
	
    public static final double DEADZONE_VALUE = 0.01;
	public static final int NUMBER_OF_CONTROLLERS = 2;

	public record SwerveModuleConfig (
			int driveMotorID,
			int rotaterMotorID,
			int encoderID,
			double rotationOffset
	) {}

    public enum Axes {
		LEFT_STICK_X(0), LEFT_STICK_Y(1), 
		LEFT_TRIGGER(2), RIGHT_TRIGGER(3), 
		RIGHT_STICK_X(4), RIGHT_STICK_Y(5);

		private final int value;

		Axes(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	public enum Buttons {
		A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(
				7), START_BUTTON(8), LEFT_STICK(9), RIGHT_STICK(10);

		private final int value;

		private Buttons(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}
}