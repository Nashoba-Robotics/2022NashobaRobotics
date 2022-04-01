package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.AccelerationControl;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/*
    Command for autonomous driving. Uses the Limelight to track objects and move towards
    them. If no object is found and spinning is enabled it will spin in place.
*/
public class AutoAimCommand extends CommandBase{
    LimelightSubsystem limelight;
    Timer noTargetTimer;
    Timer inTargetTimer;

    public AutoAimCommand(){
        addRequirements(LimelightSubsystem.getInstance());
        addRequirements(DriveSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("angle auto", 0);
        SmartDashboard.putBoolean("target?", false);

        // Choose which Limelight pipeline is used
        LimelightSubsystem.getInstance().setIntakePipeline(Constants.Limelight.REFLECTIVE_TAPE_PIPELINE);
        noTargetTimer = new Timer();
        noTargetTimer.start();
        inTargetTimer = new Timer();
        inTargetTimer.start();
    }

    @Override
    public void execute() {
        // Get the X position of the tracked object (-27 to +27)
        double tx = LimelightSubsystem.getInstance().getIntakeTx();
        // Will be set in conditional later
        double turn = 0;

        if(LimelightSubsystem.getInstance().intakeValidTarget()) {
            // If a target is found

            // if(Math.abs(tx) > 5) {
            //     // If the x position of the target is outside of the deadzone,
            //     // calculate the turn speed based on how far left/right it is
            //     turn = -tx/170;
            // }

            double txPercent = tx/27;
            if(Math.abs(txPercent) <= Constants.AUTO_AIM_DEADZONE){
                turn = 0;
            }
            else{
                turn = txPercent - Math.signum(txPercent) * Constants.AUTO_AIM_DEADZONE;
                turn *= Constants.AUTO_AIM_SENSITIVITY;
                turn = Math.pow(turn, 2);
                turn = Math.abs(turn);
                turn = Math.min(turn, 1);
                turn *= Math.signum(txPercent);
                inTargetTimer.reset();
            }
            if(!LimelightSubsystem.getInstance().intakeValidTarget()){
                noTargetTimer.reset();
            }
            else{
                noTargetTimer.stop();
            }
        }

        // Use AccelerationControl to prevent tipping
        JoystickValues joystickValues = new JoystickValues(0, turn);
        
        // Put diagnostics on Shuffleboard
        SmartDashboard.putBoolean("target?", LimelightSubsystem.getInstance().intakeValidTarget());
        SmartDashboard.putNumber("angle auto", tx);
        
        // Calculate the motor velocities using arcade drive
        MotorValues vel = JoystickProcessing.arcadeDrive(joystickValues);
        
        DriveSubsystem.getInstance().setSpeed(vel.left, vel.right);
    }

    @Override
    public boolean isFinished(){
        return noTargetTimer.get() >= 0.5 || inTargetTimer.get() >= 0.5;
    }

    @Override
    public void end(boolean interrupted){
        DriveSubsystem.getInstance().setRightMotorSpeed(0);
        DriveSubsystem.getInstance().setLeftMotorSpeed(0);
    }
}
