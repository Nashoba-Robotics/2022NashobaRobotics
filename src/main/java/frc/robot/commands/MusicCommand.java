package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MusicSubsystem;

public class MusicCommand extends CommandBase{
    
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Frequency", 0);
    }

    @Override
    public void execute(){
        double frequency = SmartDashboard.getNumber("Frequency", 0);
        MusicSubsystem.getInstance().setFrequency(frequency);
    }
}