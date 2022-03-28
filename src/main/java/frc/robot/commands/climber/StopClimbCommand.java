package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;
import frc.robot.subsystems.PusherSubsystem.PusherMotor;

public class StopClimbCommand extends CommandBase{
    public StopClimbCommand(){
        addRequirements(ClimberSubsystem.getInstance());
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void execute() {
        ClimberSubsystem.getInstance().setSpeed(0);

        PusherSubsystem.getInstance().setSpeed(PusherMotor.LEFT_PUSHER, 0);
        PusherSubsystem.getInstance().setSpeed(PusherMotor.RIGHT_PUSHER, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
