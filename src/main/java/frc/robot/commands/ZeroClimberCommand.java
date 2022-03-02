package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

public class ZeroClimberCommand extends CommandBase {
    public ZeroClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    private boolean leftFinished;
    private boolean leftEnd;
    private boolean rightFinished;
    private boolean rightEnd;

    @Override
    public void initialize() {
        leftFinished = false;
        leftEnd = false;

        rightFinished = true; // TODO: CHANGE to FALSE
        rightEnd = true;
    }

    @Override
    public void execute() {
        double leftPos = ClimberSubsystem.getInstance().getPosition(ClimberMotor.LEFT_1);
        double rightPos = ClimberSubsystem.getInstance().getPosition(ClimberMotor.RIGHT_1);

        if(leftFinished && leftPos >= 0 && leftPos <= 500){
            ClimberSubsystem.getInstance().setSpeed(ClimberMotor.LEFT_1, 0);
            leftEnd = true;
        }
        if(rightFinished && rightPos >= 0 && rightPos <= 500){
            ClimberSubsystem.getInstance().setSpeed(ClimberMotor.RIGHT_1, 0);
            rightEnd = true;
        }

        if(!ClimberSubsystem.getInstance().getLimitSwitch(ClimberMotor.LEFT_1) && !leftFinished)
            ClimberSubsystem.getInstance().setSpeed(ClimberMotor.LEFT_1, -0.05);
        else if(!leftFinished){
            ClimberSubsystem.getInstance().setSpeed(ClimberMotor.LEFT_1, 0.03);
            leftFinished = true;
        }


        if(!ClimberSubsystem.getInstance().getLimitSwitch(ClimberMotor.RIGHT_1) && !rightFinished)
            ClimberSubsystem.getInstance().setSpeed(ClimberMotor.RIGHT_1, -0.05);
        else if(!leftFinished){
            ClimberSubsystem.getInstance().setSpeed(ClimberMotor.RIGHT_1, 0.03);
            rightFinished = true;
        }

        SmartDashboard.putNumber("L 1 Pos", leftPos);
        SmartDashboard.putNumber("R 1 Pos", rightPos);
        SmartDashboard.putBoolean("L finished", leftEnd);
        SmartDashboard.putBoolean("R finished", rightEnd);
    }

    @Override
    public boolean isFinished() {
        return leftEnd && rightEnd;
        
    }

    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().stop();
    }
}
