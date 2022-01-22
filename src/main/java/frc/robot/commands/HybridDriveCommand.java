package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.lib.Units;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.Drive2019Subsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class HybridDriveCommand extends CommandBase{

    LimelightSubsystem limelight;

    public HybridDriveCommand(){
        addRequirements(LimelightSubsystem.getInstance());
        addRequirements(Drive2019Subsystem.getInstance());
        LimelightSubsystem.getInstance().setPipeline(1);
    }

    @Override
    public void initialize(){
        LimelightSubsystem.getInstance().setPipeline(1);

    }

    @Override
    public void execute(){
        double tx = LimelightSubsystem.getInstance().getTx();
        double turn = 0;
        if(Math.abs(tx) > 5 && LimelightSubsystem.getInstance().validTarget()){
            turn = tx/180;
        }else if(!LimelightSubsystem.getInstance().validTarget()){
            //System.out.println("no valid target");
            turn = 0;
        }
        double move = JoystickSubsystem.getInstance().getLeftY();
        double moveScaled = JoystickProcessing.scaleJoystick(move, Constants.MOVEMENT_DEADZONE);
        double moveShaped = JoystickProcessing.shapeJoystick(moveScaled, Constants.MOVEMENT_SENSITIVITY);
        MotorValues vel = JoystickProcessing.arcadeDrive(new JoystickValues(moveShaped, turn));
        System.out.println(Units.roundTo(moveShaped, 4) + " " + Units.roundTo(vel.left, 4) + " " + Units.roundTo(vel.right, 4));
        Drive2019Subsystem.getInstance().setSpeed(vel.left, vel.right);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        Drive2019Subsystem.getInstance().setRightMotorSpeed(0);
        Drive2019Subsystem.getInstance().setLeftMotorSpeed(0);
    }
}