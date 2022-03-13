package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoStopIntakeCommand extends CommandBase{
    public AutoStopIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(IntakeSolenoidSubsystem.getInstance());
    }
    
    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().set(0);
        IntakeSolenoidSubsystem.getInstance().setState(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
