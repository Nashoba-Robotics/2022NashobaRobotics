package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.PusherSubsystem.PusherMotor;

public class ManualPusherCommand extends CommandBase{
    public ManualPusherCommand(){
        addRequirements(PusherSubsystem.getInstance());
        addRequirements(LedSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        // Robot.enableBallLeds = false;
        LedSubsystem.getInstance().twinkle(Constants.Leds.MANUAL_PUSH);
    }
    @Override
    public void execute() {
        double speed = PusherSubsystem.getInstance().getFixedValue();
        PusherSubsystem.getInstance().setSpeed(PusherMotor.LEFT_PUSHER, speed);
        PusherSubsystem.getInstance().setSpeed(PusherMotor.RIGHT_PUSHER, speed);
    }

    @Override
    public void end(boolean interrupted) {
        PusherSubsystem.getInstance().setSpeed(PusherMotor.LEFT_PUSHER, 0);
        PusherSubsystem.getInstance().setSpeed(PusherMotor.RIGHT_PUSHER, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
