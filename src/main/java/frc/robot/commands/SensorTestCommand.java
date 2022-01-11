package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SensorSubsystem;

public class SensorTestCommand extends CommandBase{

    SensorSubsystem currSensor;

    public SensorTestCommand(){
        currSensor = new SensorSubsystem(6);
        SmartDashboard.putBoolean("DIO Output", false);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("DIO Output", currSensor.getInput());
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}