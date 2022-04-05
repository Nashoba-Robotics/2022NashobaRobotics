package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;

public class StopShooterCommand extends CommandBase {
    public StopShooterCommand() {
        addRequirements(CannonSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        CannonSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
