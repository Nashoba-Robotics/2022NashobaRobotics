package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PusherSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class ResetPusherCommand extends CommandBase{
    public ResetPusherCommand(){
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        PusherSubsystem.getInstance().resetPusher();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
