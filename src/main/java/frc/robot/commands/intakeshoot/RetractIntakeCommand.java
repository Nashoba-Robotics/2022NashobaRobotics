package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends CommandBase{
    public RetractIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override   //Tristan is bad
    public void initialize(){
        IntakeSubsystem.getInstance().retractIntake();
        IntakeSubsystem.getInstance().setIntake(0);
        IntakeSubsystem.getInstance().setGrabber(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
