package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SensorSubsystem;

public class SensorTestCommand extends CommandBase{

    SensorSubsystem currSensor;

    public SensorTestCommand(){
        currSensor = new SensorSubsystem();
        SmartDashboard.putNumber("Distance", 0);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Distance", currSensor.getDistanceSensor());
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}