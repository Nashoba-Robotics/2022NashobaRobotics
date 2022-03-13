package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends CommandBase{
    public AutoIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(IntakeSolenoidSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        IntakeSolenoidSubsystem.getInstance().setState(true);
        IntakeSubsystem.getInstance().set(0.6);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
