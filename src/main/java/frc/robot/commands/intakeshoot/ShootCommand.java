package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends CommandBase {
    public ShootCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(CannonSubsystem.getInstance());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        IntakeSubsystem.getInstance()
            .setIntake(0)
            .setGrabber(0)
            .setLoader(0.3);
        CannonSubsystem.getInstance().set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        CannonSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
