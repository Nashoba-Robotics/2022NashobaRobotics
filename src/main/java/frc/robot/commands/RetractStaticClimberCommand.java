package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

public class RetractStaticClimberCommand extends CommandBase {

    public RetractStaticClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().undeployClimb();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("L 1 Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.LEFT_1));
        SmartDashboard.putNumber("R 1 Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.RIGHT_1));
    }
}
