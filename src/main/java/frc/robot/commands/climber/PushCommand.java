package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.PusherSubsystem.PusherMotor;

public class PushCommand extends CommandBase{
    double lPos;
    double rPos;
    
    public PushCommand(){
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void execute() {
        PusherSubsystem.getInstance().push();
        lPos = PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER);
        rPos = PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER);
    }    

    @Override
    public boolean isFinished() {
        return Math.abs(lPos - Constants.Climber.PUSH_DEADZONE) >= Constants.Climber.PUSH_LEFT_POS ||
               Math.abs(rPos - Constants.Climber.PUSH_DEADZONE) >= Constants.Climber.PUSH_RIGHT_POS;
    }

}
