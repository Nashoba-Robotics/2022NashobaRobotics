package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

public class ZeroClimberCommand extends CommandBase {
    private long leftHitMillis;
    private long rightHitMillis;

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

        rightFinished = false; 
        rightEnd = false;

        leftHitMillis = Long.MAX_VALUE/2;
        rightHitMillis = Long.MAX_VALUE/2;

        ClimberSubsystem.getInstance().setLimitSwitchEnable(false);
    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("L Millis", leftHitMillis);
        // SmartDashboard.putNumber("R Millis", rightHitMillis);
        // SmartDashboard.putBoolean("L Limit", ClimberSubsystem.getInstance().getLimitSwitch(ClimberMotor.LEFT_1));
        // SmartDashboard.putBoolean("R Limit", ClimberSubsystem.getInstance().getLimitSwitch(ClimberMotor.RIGHT_1));

        long millis = System.currentTimeMillis();

        double leftPos = ClimberSubsystem.getInstance().getLeftPosition();
        double rightPos = ClimberSubsystem.getInstance().getRightPosition();

        if(leftFinished && !ClimberSubsystem.getInstance().getLeftLimitSwitch()){
            ClimberSubsystem.getInstance().setLeftSpeed(0);
            ClimberSubsystem.getInstance().zeroLeftClimber();
            leftEnd = true;
        }
        if(rightFinished && !ClimberSubsystem.getInstance().getRightLimitSwitch()){
            ClimberSubsystem.getInstance().setRightSpeed(0);
            ClimberSubsystem.getInstance().zeroRightClimber();
            rightEnd = true;
        }

        if(millis > leftHitMillis + 1000) {
            ClimberSubsystem.getInstance().setLeftSpeed(0);
            leftEnd = true;
        }
        if(millis > rightHitMillis + 1000) {
            ClimberSubsystem.getInstance().setRightSpeed(0);
            rightEnd = true;
        }

        if(!ClimberSubsystem.getInstance().getLeftLimitSwitch() && !leftFinished)
            ClimberSubsystem.getInstance().setLeftSpeed(-0.05);
        else if(!leftFinished){
            ClimberSubsystem.getInstance().setRightSpeed(0.03);
            leftFinished = true;
            leftHitMillis = millis;
        }


        if(!ClimberSubsystem.getInstance().getRightLimitSwitch() && !rightFinished)
            ClimberSubsystem.getInstance().setRightSpeed(-0.05);
        else if(!rightFinished){
            ClimberSubsystem.getInstance().setLeftSpeed(0.03);
            rightFinished = true;
            rightHitMillis = millis;
        }

        // SmartDashboard.putNumber("L 1 Pos", leftPos);
        // SmartDashboard.putNumber("R 1 Pos", rightPos);
        // SmartDashboard.putBoolean("L finished", leftEnd);
        // SmartDashboard.putBoolean("R finished", rightEnd);
    }

    @Override
    public boolean isFinished() {
        return leftEnd && rightEnd;
        
    }

    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().setLimitSwitchEnable(true);
        ClimberSubsystem.getInstance().stop();
    }
}
