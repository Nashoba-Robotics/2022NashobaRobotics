package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.AccelerationControl;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.Drive2019Subsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoDriveCommand extends CommandBase{
    LimelightSubsystem limelight;

    private AccelerationControl accelerationControl;

    public AutoDriveCommand(){
        addRequirements(LimelightSubsystem.getInstance());
        addRequirements(AbstractDriveSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("move", 0);
        SmartDashboard.putNumber("distance auto", 0);
        SmartDashboard.putBoolean("target?", false);
        SmartDashboard.putNumber("left auto", 0);
        SmartDashboard.putNumber("right auto", 0);

        accelerationControl = new AccelerationControl(
            Constants.MAX_ACCEL, Constants.MAX_DECEL, 
            Constants.MAX_ACCEL_TURN, Constants.MAX_DECEL_TURN);
    }

    @Override
    public void execute(){
        double tx = LimelightSubsystem.getInstance().getTx();
        double turn = 0;
        double move = 0;
        if(!LimelightSubsystem.getInstance().validTarget()){
            turn = 0;
            move = 0;
        } else {
            if(Math.abs(tx) > 5){
              turn = tx/400; 
            }
            if(LimelightSubsystem.getInstance().getDistanceBall() > 1){
                move = -0.05;
            }
        } 

        JoystickValues joystickValues = accelerationControl.next(new JoystickValues(move, turn));
        
        
        SmartDashboard.putNumber("move", move);
        SmartDashboard.putBoolean("target?", LimelightSubsystem.getInstance().validTarget());
        SmartDashboard.putNumber("distance auto", LimelightSubsystem.getInstance().getDistanceBall());
        
        MotorValues vel = JoystickProcessing.arcadeDrive(joystickValues);

        SmartDashboard.putNumber("left auto", vel.left);
        SmartDashboard.putNumber("left auto", vel.right);
        Drive2019Subsystem.getInstance().setSpeed(vel.left, vel.right);
    }

    @Override
    public boolean isFinished(){
        return LimelightSubsystem.getInstance().getDistanceBall() < 1;
    }

    @Override
    public void end(boolean interrupted){
        AbstractDriveSubsystem.getInstance().setRightMotorSpeed(0);
        AbstractDriveSubsystem.getInstance().setLeftMotorSpeed(0);
    }
}
