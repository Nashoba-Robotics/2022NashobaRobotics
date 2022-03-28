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
        ClimberSubsystem.getInstance().setSpeed(ClimberSubsystem.getInstance().getClimberJoystickValue());

        SmartDashboard.putNumber("L Climber Pos", ClimberSubsystem.getInstance().getLeftPosition());
        SmartDashboard.putNumber("R Climber Pos", ClimberSubsystem.getInstance().getRightPosition());
    }

    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
