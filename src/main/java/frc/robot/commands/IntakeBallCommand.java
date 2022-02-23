package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.loader.*;

public class IntakeBallCommand extends CommandBase{
    public IntakeBallCommand(){
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        IntakeSubsystem.getInstance().intake();
        GrabberSubsystem.getInstance().intake();
        LoaderSubsystem.getInstance().intake();
    }

    @Override
    public void end(boolean interrupted){
        IntakeSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
        LoaderSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
