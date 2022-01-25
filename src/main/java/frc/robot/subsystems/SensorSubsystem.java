package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase{

    private DigitalInput input;
    private AnalogInput distanceSensor;

    public SensorSubsystem(){
        //input = new DigitalInput(DIOPort);
        distanceSensor = new AnalogInput(3);
    }

    public boolean getInput(){
        return input.get();
    }

    public double getDistanceSensor() {
        return distanceSensor.getValue();
    }
}