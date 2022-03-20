package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualDeployStaticClimber extends CommandBase{
    
    @Override
    public void execute() {
        ClimberSubsystem.getInstance().manualRotatingClimb();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
