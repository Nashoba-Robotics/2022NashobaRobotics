package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake2021Subsystem;

public class Intake2021Command extends CommandBase{
    public Intake2021Command(){
        addRequirements(Intake2021Subsystem.getInstance());
    }

    @Override
    public void initialize(){
        Intake2021Subsystem.getInstance().intake(0.25);
    }

    @Override
    public void execute(){
        Intake2021Subsystem.getInstance().intake(0.25);
    }

    @Override
    public void end(boolean interrupted){
      
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
