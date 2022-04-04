package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PusherSubsystem;

public class DeployPusher extends CommandBase{
    public DeployPusher(){
        addRequirements(PusherSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        PusherSubsystem.getInstance().deployPusher();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
