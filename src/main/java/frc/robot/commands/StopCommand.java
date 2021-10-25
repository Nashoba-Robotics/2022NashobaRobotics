package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

// Stop the robot
public class StopCommand extends CommandBase {  
    public StopCommand() {
        addRequirements(DriveSubsystem.getInstance());
    }
    public void execute(){
        DriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        DriveSubsystem.getInstance().setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    } 
}
