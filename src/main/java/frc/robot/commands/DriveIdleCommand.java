package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.Units;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;

public class DriveIdleCommand extends CommandBase {
       
    public DriveIdleCommand() {
        addRequirements(DriveSubsystem.getInstance());

    }
    public void execute(){
        DriveSubsystem.getInstance().setSpeed(0, ControlMode.PercentOutput);
        
    }

}
