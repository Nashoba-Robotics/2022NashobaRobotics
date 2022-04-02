package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
    boolean finished;

    public ClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        finished = false;
        SmartDashboard.putNumber("LC Pos", 0);
        SmartDashboard.putNumber("LP Pos", 0);
        SmartDashboard.putNumber("RC Pos", 0);
        SmartDashboard.putNumber("RP Pos", 0);
        SmartDashboard.putNumber("Climber", 0);
        SmartDashboard.putNumber("Pusher", 0);
    }

    @Override
    public void execute(){
        if(ClimberSubsystem.getInstance().isCurrentBad()) {
            ClimberSubsystem.getInstance().stop();
            finished = true;
            return;
        }
        ClimberSubsystem.getInstance().setLeftSpeed(SmartDashboard.getNumber("Climber",0));
        ClimberSubsystem.getInstance().setRightSpeed(SmartDashboard.getNumber("Climber", 0));
        // ClimberSubsystem.getInstance().setSpeed(ClimberMotor.RIGHT_PUSHER, SmartDashboard.getNumber("Pusher", 0));
        // ClimberSubsystem.getInstance().setSpeed(ClimberMotor.LEFT_PUSHER, SmartDashboard.getNumber("Pusher", 0));

        SmartDashboard.putNumber("RC Stator", ClimberSubsystem.getInstance().getRightStatorCurrent());
        // SmartDashboard.putNumber("RP Stator", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.RIGHT_PUSHER));
        SmartDashboard.putNumber("LC Stator", ClimberSubsystem.getInstance().getLeftStatorCurrent());
        // SmartDashboard.putNumber("LP Stator", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.LEFT_PUSHER));

        SmartDashboard.putNumber("RC Pos", ClimberSubsystem.getInstance().getRightPosition());
        // SmartDashboard.putNumber("RP Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.RIGHT_PUSHER));
        SmartDashboard.putNumber("LC Pos", ClimberSubsystem.getInstance().getLeftPosition());
        // SmartDashboard.putNumber("LP Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.LEFT_PUSHER));

        //SmartDashboard.putBoolean("RC Switch", csi.getLimitSwitch(ClimberMotor.RIGHT_1));
        // SmartDashboard.putBoolean("L 1 Switch", csi.getLimitSwitch(ClimberMotor.LEFT_1));
    }

    @Override
    public void end(boolean interrupted){
        ClimberSubsystem.getInstance().stop();
        finished = true;
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
