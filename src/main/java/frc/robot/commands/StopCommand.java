package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

// Stops the robot
public class StopCommand extends CommandBase {  
    public StopCommand() {
        addRequirements(DriveSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        DriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        DriveSubsystem.getInstance().setSpeed(0);
    }
    public void execute(){
        DriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        DriveSubsystem.getInstance().setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return true;
    } 
}
