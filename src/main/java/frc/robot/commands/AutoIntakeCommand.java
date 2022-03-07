package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends CommandBase{
    public AutoIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().deployIntake();
        IntakeSubsystem.getInstance().setIntake(0.6);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
