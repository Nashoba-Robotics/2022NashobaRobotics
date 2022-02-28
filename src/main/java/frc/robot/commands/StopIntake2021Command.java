package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake2021Subsystem;

public class StopIntake2021Command extends CommandBase{
    public StopIntake2021Command(){
        addRequirements(Intake2021Subsystem.getInstance());
    }

    @Override
    public void initialize(){
        Intake2021Subsystem.getInstance().intake(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
