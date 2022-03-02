package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class PukeCommand extends CommandBase {
    public PukeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        IntakeSubsystem.getInstance()
            .setIntake(-0.3)
            .setGrabber(-0.3)
            .setLoader(-0.2);
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
