package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class PukeCommand extends CommandBase {
    public PukeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Puking?", true);
    }

    @Override
    public void execute() { 
        IntakeSubsystem.getInstance().set(-0.3);
        GrabberSubsystem.getInstance().set(-0.3);
        LoaderSubsystem.getInstance().set(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        SmartDashboard.putBoolean("Puking?", false);
        
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
