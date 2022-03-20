package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PusherSubsystem;

public class DeployPusher extends CommandBase{
    public DeployPusher(){
        addRequirements(PusherSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        PusherSubsystem.getInstance().deployPusher();
        SmartDashboard.putBoolean("Pusher Moved", true);
    }
    @Override
    public void execute() {
        PusherSubsystem.getInstance().deployPusher();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Pusher Moved", false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
