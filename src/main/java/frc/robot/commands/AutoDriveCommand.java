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
    private boolean spin;
    private int spinDirection;

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
        SmartDashboard.putNumber("spin?", 0);

        LimelightSubsystem.getInstance().setPipeline(0);

        spin = false;
        spinDirection = 1;

        accelerationControl = new AccelerationControl(
            Constants.MAX_ACCEL, Constants.MAX_DECEL, 
            Constants.MAX_ACCEL_TURN, Constants.MAX_DECEL_TURN);
    }

    @Override
    public void execute(){
        double tx = LimelightSubsystem.getInstance().getTx();
        double turn = 0;
        double move = 0;
        spin = SmartDashboard.getNumber("spin?", 0) == 1;
        
        // if(!LimelightSubsystem.getInstance().validTarget()){
        //     turn = 0;
        //     move = 0;
        // } else {
        //     if(Math.abs(tx) > 5){
        //       turn = tx/200; 
        //     }
        //     if(LimelightSubsystem.getInstance().getDistanceBall() > 1){
        //         move = -0.05;
        //     }
        // } 

        if(LimelightSubsystem.getInstance().validTarget()){
            if(Math.abs(tx) > 5){
                turn = tx/170;
            }
            if(LimelightSubsystem.getInstance().getDistanceBall() > Constants.SPEED_THRESHOLD_AUTO){
                move = -Constants.MOVE_SPEED_AUTO;
            }else if(LimelightSubsystem.getInstance().getDistanceBall() > Constants.MIN_DISTANCE_AUTO){
                move = -LimelightSubsystem.getInstance().getDistanceBall()/(Constants.SPEED_THRESHOLD_AUTO*(1/Constants.MOVE_SPEED_AUTO));
            }
            
            if(tx >= 0){
                spinDirection = 1;
            }else {
                spinDirection = -1;
            }
        }else if(spin){
            turn = 0.05 * spinDirection;
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
        //return LimelightSubsystem.getInstance().getDistanceBall() < 1;
        return false;
    }

    @Override
    public void end(boolean interrupted){
        AbstractDriveSubsystem.getInstance().setRightMotorSpeed(0);
        AbstractDriveSubsystem.getInstance().setLeftMotorSpeed(0);
    }
}
