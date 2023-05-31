package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LoaderSubsystem;

public class LoadCommand extends CommandBase {
    
    public LoadCommand() {
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LoaderSubsystem.getInstance().set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        LoaderSubsystem.getInstance().set(0);
    }

}
