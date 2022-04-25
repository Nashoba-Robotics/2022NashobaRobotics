package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ConfigDriveCommand extends CommandBase{
    public ConfigDriveCommand(){
        addRequirements(DriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        DriveSubsystem.getInstance().emergencyConfig();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
