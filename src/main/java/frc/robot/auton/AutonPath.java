package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.HashMap;
import java.util.Map;

public class AutonPath {
    private final PathPlannerTrajectory traj;

    private static final Map<String, Command> eventMap = new HashMap<>();

    private static final PIDConstants positionPIDConstants = new PIDConstants(
        Constants.RobotInfo.Auton.POSITION_KP,
        Constants.RobotInfo.Auton.POSITION_KI,
        0.0
    );
    private static final PIDConstants rotationPIDConstants = new PIDConstants(
        Constants.RobotInfo.Auton.ROTATION_KP,
        Constants.RobotInfo.Auton.ROTATION_KI,
        0.0
    );

    public AutonPath(String file){
        traj = PathPlanner.loadPath(file, new PathConstraints(
            Constants.RobotInfo.MAX_VELOCITY,
            Constants.RobotInfo.MAX_ACCELERATION
        ));
    }

    public Command getCommand(SwerveDriveSubsystem swerve) {
        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetPose,
            swerve.getKinematics(),
            positionPIDConstants,
            rotationPIDConstants,
            swerve::setModules,
            eventMap,
            true,
            swerve
        );

        return builder.fullAuto(traj);
    }

    public static void registerEvents (Event... events){
        for(Event event : events) registerEvent(event);
    }

    public static void registerEvent(Event event) {
        registerEvent(event.eventName, event.command);
    }

    public static void registerEvent(String eventName, Command command){
        eventMap.put(eventName, command);
    }

    public record Event(String eventName, Command command){}
}
