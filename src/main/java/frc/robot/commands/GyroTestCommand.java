package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GyroSubsystem;
/*
    Command for showing gyro information on Shuffleboard
*/
public class GyroTestCommand extends CommandBase{
    
    public GyroTestCommand(){
        addRequirements(GyroSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("GyroX", 0);
        // SmartDashboard.putNumber("GyroY", 0);
        // SmartDashboard.putNumber("GyroZ", 0);
        // SmartDashboard.putNumber("Gyro Acceleration X", 0);
        // SmartDashboard.putNumber("Gyro Acceleration Y", 0);
        // SmartDashboard.putNumber("Gyro Acceleration Z", 0);
        SmartDashboard.putNumber("angle", 0);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("angle", GyroSubsystem.getInstance().getAbsoluteAngle());
        SmartDashboard.putNumber("GyroX", GyroSubsystem.getInstance().getYawPitchRole()[0]);    //We care about this one
        // SmartDashboard.putNumber("GyroY", GyroSubsystem.getInstance().getYawPitchRole()[1]);
        // SmartDashboard.putNumber("GyroZ", GyroSubsystem.getInstance().getYawPitchRole()[2]);
        // SmartDashboard.putNumber("Gyro Acceleration X", GyroSubsystem.getInstance().getAccelX());
        // SmartDashboard.putNumber("Gyro Acceleration Y", GyroSubsystem.getInstance().getAccelY());
        // SmartDashboard.putNumber("Gyro Acceleration Z", GyroSubsystem.getInstance().getAccelZ());
        
    }

    @Override
    public void end(boolean interrupted){
        
    }


}