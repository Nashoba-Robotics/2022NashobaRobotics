package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class EjectFrontCommand extends CommandBase {
    public EjectFrontCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        IntakeSubsystem.getInstance().set(-0.3);
        GrabberSubsystem.getInstance().set(-0.3);
        LoaderSubsystem.getInstance().set(0);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
