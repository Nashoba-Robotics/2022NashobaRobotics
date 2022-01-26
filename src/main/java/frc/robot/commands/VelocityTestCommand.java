package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.AbstractDriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Drive diagnostic stuff
public class VelocityTestCommand extends CommandBase {
    public VelocityTestCommand(){
        addRequirements(AbstractDriveSubsystem.getInstance());
        SmartDashboard.putNumber("Movement%", 0);
        SmartDashboard.putNumber("Turning%", 0);
        SmartDashboard.putNumber("Vel-mode", 0);
    }

    @Override
    public void initialize() {
        AbstractDriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        AbstractDriveSubsystem.getInstance().setSpeed(0, 0);
    }

    @Override
    public void execute(){
        //Takes input from shuffle board to determine robot movement
        double movement = SmartDashboard.getNumber("Movement%", 0);
        double turning = SmartDashboard.getNumber("Turning%", 0);
        JoystickValues joystickValues = new JoystickValues(movement, turning);  //Creates an instance of JoystickValues, which will shape the values of movement and turn
        MotorValues motorValues = JoystickProcessing.arcadeDrive(joystickValues);   //We aren't using joysticks here, but we use this class because Tristan is bad at naming things

        //Sets Drive mode either to Velocity mode or Percent mode
        double velMode = SmartDashboard.getNumber("Vel-mode", 0);
        if(velMode != 0) {
            AbstractDriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        } else {
            AbstractDriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        }

        //Takes left and right velocities from SmartDashboards and uses it to set the motor velocities
        double leftVel = motorValues.left;
        double rightVel = motorValues.right;
        SmartDashboard.putNumber("Left Value", leftVel);
        SmartDashboard.putNumber("Right Value", rightVel);
        AbstractDriveSubsystem.getInstance().setSpeed(leftVel, rightVel);

        //Motor diagnostic stuff
        SmartDashboard.putNumber("L-velocity", AbstractDriveSubsystem.getInstance().getLeftMotorVelocity());
        SmartDashboard.putNumber("R-velocity", AbstractDriveSubsystem.getInstance().getRightMotorVelocity());
        SmartDashboard.putNumber("L-error", AbstractDriveSubsystem.getInstance().getLeftMotorError());
        SmartDashboard.putNumber("R-error", AbstractDriveSubsystem.getInstance().getRightMotorError());
    }

    @Override
    public void end(boolean interrupted) {
        //Sets motor speeds to 0, so the robot stops when the command ends
        AbstractDriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        AbstractDriveSubsystem.getInstance().setSpeed(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}