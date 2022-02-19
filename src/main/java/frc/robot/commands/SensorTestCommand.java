package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SensorSubsystem;

//Sensor diagnostic stuff
public class SensorTestCommand extends CommandBase{

    SensorSubsystem sensor2;
    SensorSubsystem sensor3;
    SensorSubsystem sensor4;
    SensorSubsystem sensor5;
    SensorSubsystem sensor6;

    public SensorTestCommand(){
        sensor2 = new SensorSubsystem(3);
        // sensor3 = new SensorSubsystem(3);
        // sensor4 = new SensorSubsystem(4);
        // sensor5 = new SensorSubsystem(5);
        // sensor6 = new SensorSubsystem(6);
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("sensor 2", sensor2.getInput());
        // SmartDashboard.putBoolean("sensor 3", sensor3.getInput());
        // SmartDashboard.putBoolean("sensor 4", sensor4.getInput());
        // SmartDashboard.putBoolean("sensor 5", sensor5.getInput());
        // SmartDashboard.putBoolean("sensor 6", sensor6.getInput());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}