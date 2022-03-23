package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

public class ManualClimberCommand extends CommandBase{
    public ManualClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void execute() {
        ClimberSubsystem.getInstance().setSpeed(ClimberMotor.LEFT_CLIMBER, ClimberSubsystem.getInstance().getClimberJoystickValue());
        ClimberSubsystem.getInstance().setSpeed(ClimberMotor.RIGHT_CLIMBER, ClimberSubsystem.getInstance().getClimberJoystickValue());

        SmartDashboard.putNumber("L Climber Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.LEFT_CLIMBER));
        SmartDashboard.putNumber("R Climber Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.RIGHT_CLIMBER));
    }

    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().setSpeed(ClimberMotor.LEFT_CLIMBER, 0);
        ClimberSubsystem.getInstance().setSpeed(ClimberMotor.RIGHT_CLIMBER, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
