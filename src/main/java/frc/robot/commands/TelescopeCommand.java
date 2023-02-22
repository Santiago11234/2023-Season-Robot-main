package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeCommand extends CommandBase {
    private final TelescopeSubsystem telescope;
    private final OI oi;
    public TelescopeCommand(TelescopeSubsystem telescope, OI oi){
        this.telescope = telescope;
        this.oi = oi;
        addRequirements(telescope);
    }
    @Override
    public void initialize() {
        telescope.extend(0);
    }
    @Override
    public void execute() {
        double position = oi.getAxis(0, Constants.Axes.RIGHT_STICK_Y);

        telescope.extend(position);
        // telescope.setPosition(Position);
    }
}