package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class IntakePracticeCommand extends CommandBase{
    public IntakePracticeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().set(0.75);
        GrabberSubsystem.getInstance().set(0.4);
        LoaderSubsystem.getInstance().set(0.5);
        CannonSubsystem.getInstance().set(0.1);
        

    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
        LoaderSubsystem.getInstance().set(0);
        CannonSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
