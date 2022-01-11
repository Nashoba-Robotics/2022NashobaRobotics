package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.Drive2019Subsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AimBallCommand extends CommandBase{

    LimelightSubsystem limelight;
    AbstractDriveSubsystem driveTrain;

    public AimBallCommand(){
        addRequirements(LimelightSubsystem.getInstance());
        addRequirements(Drive2019Subsystem.getInstance());
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double tx = LimelightSubsystem.getInstance().getTx();
        double turn = 0;
        if(Math.abs(tx) > 5 && LimelightSubsystem.getInstance().validTarget()){
            turn = tx/180;
        }
        double move = JoystickSubsystem.getInstance().getLeftY();
        double moveScaled = JoystickProcessing.scaleJoystick(move, Constants.MOVEMENT_DEADZONE);
        double moveShaped = JoystickProcessing.shapeJoystick(moveScaled, Constants.MOVEMENT_SENSITIVITY);
        MotorValues vel = JoystickProcessing.arcadeDrive(new JoystickValues(moveShaped, turn));
        Drive2019Subsystem.getInstance().setSpeed(vel.left, vel.right);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        driveTrain.setRightMotorSpeed(0);
        driveTrain.setLeftMotorSpeed(0);
    }
}