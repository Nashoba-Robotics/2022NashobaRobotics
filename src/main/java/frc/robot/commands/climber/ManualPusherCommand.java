package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PusherSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;
import frc.robot.subsystems.PusherSubsystem.PusherMotor;

public class ManualPusherCommand extends CommandBase{
    public ManualPusherCommand(){
        addRequirements(PusherSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Left Pusher NU", PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("Right Pusher NU", PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER));
        SmartDashboard.putNumber("Left Pusher Current", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("Right Pusher Current", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.RIGHT_PUSHER));
        SmartDashboard.putNumber("Left Climber Currect", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.LEFT_CLIMBER));
        SmartDashboard.putNumber("Right Climber Current", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.RIGHT_CLIMBER));
    }
    @Override
    public void execute() {
        double speed = PusherSubsystem.getInstance().getFixedValue();
        PusherSubsystem.getInstance().setSpeed(PusherMotor.LEFT_PUSHER, speed);
        PusherSubsystem.getInstance().setSpeed(PusherMotor.RIGHT_PUSHER, speed);

        SmartDashboard.putNumber("Left Pusher NU", PusherSubsystem.getInstance().getPosition(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("Right Pusher NU", PusherSubsystem.getInstance().getPosition(PusherMotor.RIGHT_PUSHER));

        SmartDashboard.putNumber("Left Pusher Current", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.LEFT_PUSHER));
        SmartDashboard.putNumber("Right Pusher Current", PusherSubsystem.getInstance().getStatorCurrent(PusherMotor.RIGHT_PUSHER));

        SmartDashboard.putNumber("Left Climber Currect", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.LEFT_CLIMBER));
        SmartDashboard.putNumber("Right Climber Current", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.RIGHT_CLIMBER));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
