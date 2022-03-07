package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoStopIntakeCommand extends CommandBase{
    public AutoStopIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
    }
    
    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setIntake(0);
        IntakeSubsystem.getInstance().retractIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
