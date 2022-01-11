package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.AbstractDriveSubsystem.DriveMode;

// Stop the robot
public class LimelightCommand extends CommandBase {  
    public LimelightCommand() {
        addRequirements(LimelightSubsystem.getInstance());
        SmartDashboard.putNumber("LED", 0);
        SmartDashboard.putNumber("ta", 0);
        SmartDashboard.putNumber("tx", 0);
        SmartDashboard.putNumber("ty", 0);
        SmartDashboard.putNumber("h2", 0);
    }
    public void execute(){
        LimelightSubsystem.getInstance().setLed(SmartDashboard.getNumber("LED", 0));
        double h2 = SmartDashboard.getNumber("h2", 0);
        SmartDashboard.putNumber("ta", LimelightSubsystem.getInstance().getTa());
        SmartDashboard.putNumber("tx", LimelightSubsystem.getInstance().getTx());
        SmartDashboard.putNumber("ty", LimelightSubsystem.getInstance().getTy());
        SmartDashboard.putNumber("distance", LimelightSubsystem.getInstance().getDistanceBall());
    }
    @Override
    public boolean isFinished() {
        return false;
    } 
}
