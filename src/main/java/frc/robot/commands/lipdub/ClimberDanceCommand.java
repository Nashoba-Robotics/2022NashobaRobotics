package frc.robot.commands.lipdub;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDanceCommand extends CommandBase{
    final int leftUpPos = Constants.Climber.DEPLOY_LEFT_POS;
    final int leftDownPos = 20_000;
    final int rightUpPos = Constants.Climber.DEPLOY_RIGHT_POS;;
    final int rightDownPos = 20_000;

    boolean leftDown = false;
    boolean rightDown = true;

    @Override
    public void execute() {
        if(leftDown) ClimberSubsystem.getInstance().leftSetPos(leftDownPos);
        else ClimberSubsystem.getInstance().leftSetPos(leftUpPos);

        if(rightDown) ClimberSubsystem.getInstance().rightSetPos(rightDownPos);
        else ClimberSubsystem.getInstance().rightSetPos(rightUpPos);
        
        if(Math.abs(ClimberSubsystem.getInstance().getLeftPosition() - leftUpPos) < 1000) leftDown = true;
        else if(Math.abs(ClimberSubsystem.getInstance().getLeftPosition() - leftDownPos) < 1000) leftDown = false;

        if(Math.abs(ClimberSubsystem.getInstance().getRightPosition() - rightUpPos) < 1000) rightDown = true;
        else if(Math.abs(ClimberSubsystem.getInstance().getRightPosition() - rightDownPos) <1000) rightDown = false;

        SmartDashboard.putNumber("Left Climber Pos", ClimberSubsystem.getInstance().getLeftPosition());
        SmartDashboard.putNumber("Right Climber Pos", ClimberSubsystem.getInstance().getRightPosition());

        SmartDashboard.putBoolean("Left Down?", leftDown);
        SmartDashboard.putBoolean("Right Down?", rightDown);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
