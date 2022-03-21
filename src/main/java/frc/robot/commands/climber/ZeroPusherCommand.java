package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PusherSubsystem;

public class ZeroPusherCommand extends CommandBase{

    public ZeroPusherCommand(){
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        PusherSubsystem.getInstance().zeroPushers();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
