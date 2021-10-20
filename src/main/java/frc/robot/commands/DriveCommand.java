package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.Units;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;

// TODO rename -- more specific to joysticks
public class DriveCommand extends CommandBase {
    public enum DriveMode {
        VELOCITY, PERCENT;
    }

    private DriveMode drive_mode;

    // mode: either DriveMode.VELOCITY for velocity
    // control or DriveMode.PERCENT for percent output control
    public DriveCommand(DriveMode mode) {
        this.drive_mode = mode;
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(JoystickSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        DriveSubsystem.getInstance().setSpeed(0, 0, ControlMode.Velocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    // TODO move percent/velocity logic into DriveSubsystem, add method to set mode
    // TODO change from length 2 arrays to dedicated classes
    // TODO add options to enable/disable shuffleboard diagnostics, reorganize
    @Override
    public void execute() { //10/19/21 Joysticks output reverse values
        // rightX: turning joystick
        double rightX = JoystickSubsystem.getInstance().getRightX();
        // leftY: movement joystick
        double leftY = JoystickSubsystem.getInstance().getLeftY();
        double[] speeds = JoystickProcessing.processJoysticks(leftY, rightX);
       //double[] speeds = {leftY, leftY};
        
        if(drive_mode == DriveMode.PERCENT) {
            DriveSubsystem.getInstance().setSpeed(speeds[0], speeds[1], ControlMode.PercentOutput);
        } else {
            double leftVel = Units.percent2Velocity(speeds[0]);
            //double leftVel = Units.percent2Velocity(rightX);
            double rightVel = Units.percent2Velocity(speeds[1]);
            //double rightVel = Units.percent2Velocity(leftY);
            DriveSubsystem.getInstance().setSpeed(leftVel, rightVel, ControlMode.Velocity);
        }
        SmartDashboard.putNumber("Joystick Left Y", leftY);
        SmartDashboard.putNumber("Joystick Right X", rightX);
        SmartDashboard.putNumber("Left percent", speeds[0]);
        SmartDashboard.putNumber("Right percent", speeds[1]);
        //Puts the velocity that the motor controller is reading to SmartDashboard
        SmartDashboard.putNumber("Left Motor Real Velocity", DriveSubsystem.getInstance().getLeftMotorVelocity());
        SmartDashboard.putNumber("Right Motor Real Velocity", DriveSubsystem.getInstance().getRightMotorVelocity());

        // SmartDashboard.putNumber("Left Motor Real Velocity", PercentOutputDriveSubsystem.getInstance().getLeftMotorVelocity());
        // SmartDashboard.putNumber("Right Motor Real Velocity", PercentOutputDriveSubsystem.getInstance().getRightMotorVelocity());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().setSpeed(0, 0, ControlMode.Velocity);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
