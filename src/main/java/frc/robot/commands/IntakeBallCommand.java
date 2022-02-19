package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.loader.*;

public class IntakeBallCommand extends CommandBase{
    public IntakeBallCommand(){

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        IntakeSubsystem.getInstance().intake();
        LoaderSubsystem.getInstance().intake();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
