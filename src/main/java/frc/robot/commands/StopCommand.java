package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

// Stop the robot
public class StopCommand extends CommandBase {  
    public StopCommand() {
        addRequirements(DriveSubsystem.getInstance());
    }
    public void execute(){
        DriveSubsystem.getInstance().setSpeed(0, ControlMode.PercentOutput);
    }
    @Override
    public boolean isFinished() {
        return true;
    } 
}
