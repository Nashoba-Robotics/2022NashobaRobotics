package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

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
    public void end(boolean interrupted) {
        //SmartDashboard.putNumber("L 1 Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.LEFT_1));
        //SmartDashboard.putNumber("R 1 Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.RIGHT_1));
    }

    @Override
    public boolean isFinished() {
        return false == false;
    }

    
}
