package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.RenameThisLater;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class PukeCommand extends CommandBase {
    private RenameThisLater thing;
    public PukeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Puking?", true);
        thing = new RenameThisLater(GrabberSubsystem.getInstance().getPercentOutput(), -0.25, 0.3);
        thing.start();
    }

    @Override
    public void execute() { 
        IntakeSubsystem.getInstance().set(-0.4);
        GrabberSubsystem.getInstance().set(thing.get());
        LoaderSubsystem.getInstance().set(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
        SmartDashboard.putBoolean("Puking?", false);
        
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
