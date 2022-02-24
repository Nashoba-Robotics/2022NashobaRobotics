package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class UndeployIntakeCommand extends CommandBase{
    public UndeployIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        IntakeSubsystem.getInstance().retractIntake();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
