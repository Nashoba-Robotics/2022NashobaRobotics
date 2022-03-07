package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class PukeCommand extends CommandBase {
    public PukeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Puking?", true);
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
        SmartDashboard.putBoolean("Puking?", false);
        
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
