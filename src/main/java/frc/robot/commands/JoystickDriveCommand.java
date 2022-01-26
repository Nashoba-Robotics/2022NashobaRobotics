package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.AccelerationControl;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.Units;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.AbstractDriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
    Operate the robot using the joysticks.
*/
public class JoystickDriveCommand extends CommandBase {

    // Toggle between arcade drive (true) and radius drive (false)
    private boolean arcadeDrive;    
    // Record the state of the joystick triggers to detect when they
    // are pressed/released
    private boolean buttonPressedLeft;
    private boolean lastPressedLeft;
    private boolean buttonPressedRight;
    private boolean lastPressedRight;

    private boolean lastPressedbottom;
    private boolean bottomPressed;

    // The left and right joystick triggers
    private JoystickButton joystickTriggerLeft;
    private JoystickButton joystickTriggerRight;

    private JoystickButton joystickBottomRight;

    private AccelerationControl accelerationControl;

    public JoystickDriveCommand() {
        addRequirements(AbstractDriveSubsystem.getInstance());
        addRequirements(JoystickSubsystem.getInstance());
        joystickTriggerLeft = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 1);
        joystickTriggerRight = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), 1);
        joystickBottomRight = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), 2);
    }

    @Override
    public void initialize() {
        AbstractDriveSubsystem.getInstance().setSpeed(0, 0);
        // Default to enabling arcade drive
        arcadeDrive = true;
        // Joystick buttons start off unpressed
        buttonPressedLeft = joystickTriggerLeft.get();
        lastPressedLeft = joystickTriggerLeft.get();
        buttonPressedRight = joystickTriggerRight.get();
        lastPressedRight = joystickTriggerRight.get();

        accelerationControl = new AccelerationControl(
            Constants.MAX_ACCEL, Constants.MAX_DECEL, 
            Constants.MAX_ACCEL_TURN, Constants.MAX_DECEL_TURN);
    }

    @Override
    public void execute() {
        // Joystick drive uses velocity mode
        AbstractDriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        
        // Toggle arcade drive when the left trigger is pressed
        buttonPressedLeft = joystickTriggerLeft.get();
        if(buttonPressedLeft && lastPressedLeft != buttonPressedLeft) arcadeDrive = !arcadeDrive;
        lastPressedLeft = buttonPressedLeft;

        // Enable break mode when the right trigger is pressed
        buttonPressedRight = joystickTriggerRight.get();
        if(buttonPressedRight && lastPressedRight != buttonPressedRight) AbstractDriveSubsystem.getInstance().changeBrakeMode();
        lastPressedRight = buttonPressedRight;

        bottomPressed = joystickBottomRight.get();
        if(bottomPressed && lastPressedbottom != bottomPressed){
            CommandScheduler.getInstance().schedule(RobotContainer.hybridDriveCommand);
            CommandScheduler.getInstance().run();
            CommandScheduler.getInstance().cancel(RobotContainer.joystickDriveCommand);
        }
        lastPressedbottom = bottomPressed;
        
        // Show whether break mode is enabled
        SmartDashboard.putBoolean("Brake Mode", AbstractDriveSubsystem.getInstance().getBrakeMode());

        // right joystick x position - turning
        double rightX = JoystickSubsystem.getInstance().getRightX();
        // left joystick y position - movement
        double leftY = JoystickSubsystem.getInstance().getLeftY();
        // left joystick x position - h-drive
        double leftX = JoystickSubsystem.getInstance().getLeftX();
        // creates a JoystickValues object to represent the output from the actual joysticks; before all processing
        JoystickValues joystickValues = new JoystickValues(leftY, rightX);
        // adds deadzone to joystick values while also keeping graph continuous
        joystickValues = JoystickProcessing.scaleJoysticks(joystickValues);
        // adds shaping to joystick; see JoystickProcessing
        joystickValues = JoystickProcessing.shapeJoysticks(joystickValues);

        // changes joystickValues if exceeds acceleration limit
        joystickValues = accelerationControl.next(joystickValues);

        //Switches between Arcade Drive and Radius Drive
        //motorValues are move/turn instead of left/right
        MotorValues motorValues;
        if(arcadeDrive){
            motorValues = JoystickProcessing.arcadeDrive(joystickValues);
        } 
        else{ 
            motorValues = JoystickProcessing.radiusDrive(joystickValues);
        }

        SmartDashboard.putBoolean("ArcadeDrive on?", arcadeDrive);
                
        //Sets the speed
        AbstractDriveSubsystem.getInstance().setSpeed(motorValues.left, motorValues.right);
        //H-Drive
        AbstractDriveSubsystem.getInstance().setHDriveSpeed(leftX);
        
        //Diagnostic values for Joysticks and motors
        SmartDashboard.putNumber("Joystick Left Y", leftY);
        SmartDashboard.putNumber("Joystick Right X", rightX);
        SmartDashboard.putNumber("Left percent", motorValues.left);
        SmartDashboard.putNumber("Right percent", motorValues.right);
        //Puts the velocity that the motor controller is reading to SmartDashboard
        SmartDashboard.putNumber("Left Motor Real Velocity", AbstractDriveSubsystem.getInstance().getLeftMotorVelocity());
        SmartDashboard.putNumber("Right Motor Real Velocity", AbstractDriveSubsystem.getInstance().getRightMotorVelocity());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //When this command ends, set everything to 0 and gets out of Brake Mode
        AbstractDriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        AbstractDriveSubsystem.getInstance().setSpeed(0, 0);
        AbstractDriveSubsystem.getInstance().setBrakeMode(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //Command is constantly running, and can only be stopped manually
        return false;
    }
}
