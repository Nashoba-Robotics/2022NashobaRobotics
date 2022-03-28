package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.PusherSubsystem.PusherMotor;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class ResetPusherCommand extends CommandBase{
    private double lPos;
    private double rPos;
    public ResetPusherCommand(){
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void execute() {
        lPos = PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER);
        rPos = PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER);
        PusherSubsystem.getInstance().resetPusher();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lPos) <= Constants.Climber.RESET_DEADZONE
        || Math.abs(rPos) <= Constants.Climber.RESET_DEADZONE;
    }

}
