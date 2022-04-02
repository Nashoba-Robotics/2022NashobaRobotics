package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PusherSubsystem;

public class DeployClimberCommad extends CommandBase{
    public DeployClimberCommad(){
        addRequirements(ClimberSubsystem.getInstance());
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        Robot.enableBallLeds = false;
        LedSubsystem.getInstance().twinkle(Constants.Leds.DEPLOY_CLIMBER);
        ClimberSubsystem.getInstance().deployClimber();
        PusherSubsystem.getInstance().resetPusher();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
