package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.Units;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

public class JoystickDriveCommand extends CommandBase {

    // mode: either DriveMode.VELOCITY for velocity
    // control or DriveMode.PERCENT for percent output control
    public JoystickDriveCommand() {
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(JoystickSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        DriveSubsystem.getInstance().setSpeed(0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    // TODO change from length 2 arrays to dedicated classes
    // TODO add options to enable/disable shuffleboard diagnostics, reorganize
    @Override
    public void execute() { //10/19/21 Joysticks output reverse values
        // rightX: turning joystick
        double rightX = JoystickSubsystem.getInstance().getRightX();
        // leftY: movement joystick
        double leftY = JoystickSubsystem.getInstance().getLeftY();
        JoystickValues joystickValues = new JoystickValues(leftY, rightX);
        MotorValues motorValues = JoystickProcessing.processJoysticks(joystickValues);
        
       //double[] speeds = {leftY, leftY};
        
        DriveSubsystem.getInstance().setSpeed(motorValues.left, motorValues.right);
        
        SmartDashboard.putNumber("Joystick Left Y", leftY);
        SmartDashboard.putNumber("Joystick Right X", rightX);
        SmartDashboard.putNumber("Left percent", motorValues.left);
        SmartDashboard.putNumber("Right percent", motorValues.right);
        //Puts the velocity that the motor controller is reading to SmartDashboard
        SmartDashboard.putNumber("Left Motor Real Velocity", DriveSubsystem.getInstance().getLeftMotorVelocity());
        SmartDashboard.putNumber("Right Motor Real Velocity", DriveSubsystem.getInstance().getRightMotorVelocity());

        // SmartDashboard.putNumber("Left Motor Real Velocity", PercentOutputDriveSubsystem.getInstance().getLeftMotorVelocity());
        // SmartDashboard.putNumber("Right Motor Real Velocity", PercentOutputDriveSubsystem.getInstance().getRightMotorVelocity());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        DriveSubsystem.getInstance().setSpeed(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
