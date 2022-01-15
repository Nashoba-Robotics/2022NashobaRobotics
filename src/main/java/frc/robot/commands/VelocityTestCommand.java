package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.lib.Units;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AbstractDriveSubsystem.DriveMode;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Simple debugging inteface for testing the robot at 
// specific set inputs (instead of joysticks, which are
// hard to control precisely)
public class VelocityTestCommand extends CommandBase {

    public VelocityTestCommand(){
        addRequirements(DriveSubsystem.getInstance());
        SmartDashboard.putNumber("Movement%", 0);
        SmartDashboard.putNumber("Turning%", 0);
        SmartDashboard.putNumber("Vel-mode", 0);
    }

    @Override
    public void initialize() {
        DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        DriveSubsystem.getInstance().setSpeed(0, 0);
    }

    @Override
    public void execute(){
        double movement = SmartDashboard.getNumber("Movement%", 0);
        double turning = SmartDashboard.getNumber("Turning%", 0);
        JoystickValues joystickValues = new JoystickValues(movement, turning);
        MotorValues motorValues = JoystickProcessing.arcadeDrive(joystickValues);
        double velMode = SmartDashboard.getNumber("Vel-mode", 0);
        
        if(velMode != 0) {
            DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        } else {
            DriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        }

        //System.out.println(DriveSubsystem.getInstance().getDriveMode());


        double leftVel = motorValues.left;
        double rightVel = motorValues.right;
        SmartDashboard.putNumber("Left Value", leftVel);
        SmartDashboard.putNumber("Right Value", rightVel);

        DriveSubsystem.getInstance().setSpeed(leftVel, rightVel);

        SmartDashboard.putNumber("L-velocity", DriveSubsystem.getInstance().getLeftMotorVelocity());
        SmartDashboard.putNumber("R-velocity", DriveSubsystem.getInstance().getRightMotorVelocity());
        System.out.println(System.currentTimeMillis() + "," + DriveSubsystem.getInstance().getLeftMotorVelocity() + "," + DriveSubsystem.getInstance().getRightMotorVelocity());
        SmartDashboard.putNumber("L-error", DriveSubsystem.getInstance().getLeftMotorError());
        SmartDashboard.putNumber("R-error", DriveSubsystem.getInstance().getRightMotorError());
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        DriveSubsystem.getInstance().setSpeed(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}