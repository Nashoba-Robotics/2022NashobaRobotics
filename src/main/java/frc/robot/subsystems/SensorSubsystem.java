package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase{

    private DigitalInput input;

    public SensorSubsystem(int DIOPort){
        input = new DigitalInput(DIOPort);
    }

    public boolean getInput(){
        return input.get();
    }
}