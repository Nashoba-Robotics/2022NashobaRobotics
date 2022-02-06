package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase{

    private DigitalInput input;

    public SensorSubsystem(int port){
        input = new DigitalInput(port);
    }

    public boolean getInput(){
        return input.get();
    }
}