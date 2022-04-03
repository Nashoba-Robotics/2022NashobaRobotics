package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PusherSubsystem;

public class DeployClimberCommand extends CommandBase {

    public DeployClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
        addRequirements(PusherSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        
        // Robot.enableBallLeds = false;
        // LedSubsystem.getInstance().twinkle(Constants.Leds.DEPLOY_CLIMBER);
        ClimberSubsystem.getInstance().deployClimber();
        PusherSubsystem.getInstance().resetPusher();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
