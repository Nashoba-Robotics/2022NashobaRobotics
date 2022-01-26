package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

//Limelight diagnostic stuff
public class LimelightCommand extends CommandBase {  
    public LimelightCommand() {
        addRequirements(LimelightSubsystem.getInstance());
        SmartDashboard.putNumber("ta", 0);
        SmartDashboard.putNumber("tx", 0);
        SmartDashboard.putNumber("ty", 0);
        SmartDashboard.putNumber("h2", 0);
    }
    public void execute(){
        SmartDashboard.putNumber("ta", LimelightSubsystem.getInstance().getTa());   //Area of the target
        SmartDashboard.putNumber("tx", LimelightSubsystem.getInstance().getTx());   //X position of the target
        SmartDashboard.putNumber("ty", LimelightSubsystem.getInstance().getTy());   //Y position of the target
        SmartDashboard.putNumber("distance", LimelightSubsystem.getInstance().getDistanceBall());   //Distance from the ball
    }
    @Override
    public boolean isFinished() {
        return false;
    } 
}
