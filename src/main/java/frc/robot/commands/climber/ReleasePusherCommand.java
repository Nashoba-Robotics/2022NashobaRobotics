package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.PusherSubsystem.PusherMotor;

public class ReleasePusherCommand extends CommandBase{
    double lPos;
    double rPos;
    public ReleasePusherCommand(){
        addRequirements(PusherSubsystem.getInstance());
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().twinkle(Constants.Leds.TEMPORARY_RELEASE);
    }

    @Override
    public void execute() {
        lPos = PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER);
        rPos = PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER);
        PusherSubsystem.getInstance().releasePusherSlow();
    }

    @Override
    public void end(boolean interrupted) {
        PusherSubsystem.getInstance().releasePusherFast();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Constants.Climber.RELEASE_LEFT_PUSHER_SLOW_POS - lPos) <= Constants.Climber.RELEASE_DEADZONE
        || Math.abs(Constants.Climber.RELEASE_RIGHT_PUSHER_SLOW_POS - rPos) <= Constants.Climber.RELEASE_DEADZONE;
    }
}
