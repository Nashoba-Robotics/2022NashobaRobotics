package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PusherSubsystem;

public class ReleasePusherCommand extends CommandBase{
    public ReleasePusherCommand(){
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
