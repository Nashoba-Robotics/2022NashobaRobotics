package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

//Limelight diagnostic stuff
public class LimelightCommand extends CommandBase {  
    public LimelightCommand() {
        addRequirements(LimelightSubsystem.getInstance());
    }

    public void execute(){
        SmartDashboard.putNumber("tx shooter", LimelightSubsystem.getInstance().getShooterTx());
        SmartDashboard.putNumber("ty shooter", LimelightSubsystem.getInstance().getShooterTy());
        SmartDashboard.putNumber("d shooter", LimelightSubsystem.getInstance().getDistanceShooter());
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}
