package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.AbstractDriveSubsystem.DriveMode;

// Stops the robot
public class StopCommand extends CommandBase {  
    public StopCommand() {
        addRequirements(AbstractDriveSubsystem.getInstance());
    }
    public void execute(){
        AbstractDriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        AbstractDriveSubsystem.getInstance().setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    } 
}
