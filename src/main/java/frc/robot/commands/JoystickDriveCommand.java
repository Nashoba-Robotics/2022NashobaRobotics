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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
    Operate the robot using the joysticks.
*/
public class JoystickDriveCommand extends CommandBase {

    // Toggle between arcade drive (true) and radius drive (false)
    private boolean arcadeDrive;    
    private boolean invertDrive;
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

    //private AccelerationControl accelerationControl;

    public JoystickDriveCommand() {
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(JoystickSubsystem.getInstance());
        joystickTriggerLeft = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 1);
        joystickTriggerRight = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), 1);
        joystickBottomRight = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), 2);
    }

    @Override
    public void initialize() {
        DriveSubsystem.getInstance().setSpeed(0, 0);
        // Default to enabling arcade drive
        arcadeDrive = true;
        invertDrive = false;
        // Joystick buttons start off unpressed
        buttonPressedLeft = joystickTriggerLeft.get();
        lastPressedLeft = joystickTriggerLeft.get();
        buttonPressedRight = joystickTriggerRight.get();
        lastPressedRight = joystickTriggerRight.get();

        // accelerationControl = new AccelerationControl(
        //     Constants.MAX_ACCEL, Constants.MAX_DECEL, 
        //     Constants.MAX_ACCEL_TURN, Constants.MAX_DECEL_TURN);

        SmartDashboard.putNumber("distanceX", 0);
        SmartDashboard.putNumber("distanceY", 0);
    }

    @Override
    public void execute() {
        // Joystick drive uses velocity mode
        DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        
        // Toggle arcade drive when the left trigger is pressed
        //Temporarily switch left trigger to invert drive
        buttonPressedLeft = joystickTriggerLeft.get();
        if(buttonPressedLeft && lastPressedLeft != buttonPressedLeft){
            //arcadeDrive = !arcadeDrive;
            invertDrive = !invertDrive;
            // accelerationControl.invert();
        } 
        lastPressedLeft = buttonPressedLeft;

        // Enable break mode when the right trigger is pressed
        buttonPressedRight = joystickTriggerRight.get();
        if(buttonPressedRight && lastPressedRight != buttonPressedRight) DriveSubsystem.getInstance().changeBrakeMode();
        lastPressedRight = buttonPressedRight;

        bottomPressed = joystickBottomRight.get();
        if(bottomPressed && lastPressedbottom != bottomPressed){
            CommandScheduler.getInstance().schedule(RobotContainer.hybridDriveCommand);
            CommandScheduler.getInstance().run();
            CommandScheduler.getInstance().cancel(RobotContainer.joystickDriveCommand);
        }
        lastPressedbottom = bottomPressed;
        
        // Show whether break mode is enabled
        SmartDashboard.putBoolean("Brake Mode", DriveSubsystem.getInstance().getBrakeMode());

        // right joystick x position - turning
        double rightX = JoystickSubsystem.getInstance().getRightX();
        // left joystick y position - movement
        double leftY = JoystickSubsystem.getInstance().getLeftY();
        // creates a JoystickValues object to represent the output from the actual joysticks; before all processing
        JoystickValues joystickValues = new JoystickValues(leftY, rightX);
        // adds deadzone to joystick values while also keeping graph continuous
        joystickValues = JoystickProcessing.scaleJoysticks(joystickValues);
        // adds shaping to joystick; see JoystickProcessing
        joystickValues = JoystickProcessing.shapeJoysticks(joystickValues);

        // changes joystickValues if exceeds acceleration limit
        //joystickValues = accelerationControl.next(joystickValues);

        //Switches between Arcade Drive and Radius Drive
        //motorValues are move/turn instead of left/right
        MotorValues motorValues;
        if(arcadeDrive){
            motorValues = JoystickProcessing.arcadeDrive(joystickValues, invertDrive);
        } 
        else{ 
            motorValues = JoystickProcessing.radiusDrive(joystickValues);
        }

        SmartDashboard.putBoolean("ArcadeDrive on?", arcadeDrive);
        SmartDashboard.putBoolean("InvertedDrive?", invertDrive);
                
        //Sets the speed
        if(!invertDrive) DriveSubsystem.getInstance().setSpeed(motorValues.left, motorValues.right);
        else DriveSubsystem.getInstance().setSpeed(-motorValues.left, -motorValues.right);
        
        // //Diagnostic values for Joysticks and motors
        // SmartDashboard.putNumber("Joystick Left Y", leftY);
        // SmartDashboard.putNumber("Joystick Right X", rightX);
        // SmartDashboard.putNumber("Left percent", motorValues.left);
        // SmartDashboard.putNumber("Right percent", motorValues.right);
        // //Puts the velocity that the motor controller is reading to SmartDashboard
        // SmartDashboard.putNumber("Left Motor Real Velocity", DriveSubsystem.getInstance().getLeftMotorVelocity());
        // SmartDashboard.putNumber("Right Motor Real Velocity", DriveSubsystem.getInstance().getRightMotorVelocity());
        // SmartDashboard.putNumber("leftCurrent", DriveSubsystem.getInstance().getLeftMotorCurrent());
        // SmartDashboard.putNumber("rightCurrent", DriveSubsystem.getInstance().getRightMotorCurrent());
        SmartDashboard.putNumber("distanceX", DriveSubsystem.getInstance().getTranslationX());
        SmartDashboard.putNumber("distanceY", DriveSubsystem.getInstance().getTranslationY());
        SmartDashboard.putNumber("robotAngle", DriveSubsystem.getInstance().getAngle());
        SmartDashboard.putNumber("rightDistance", DriveSubsystem.getInstance().getDistanceLeft());
        SmartDashboard.putNumber("leftDistance", DriveSubsystem.getInstance().getDistanceRight());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //When this command ends, set everything to 0 and gets out of Brake Mode
        DriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        DriveSubsystem.getInstance().setSpeed(0, 0);
        DriveSubsystem.getInstance().setBrakeMode(false);
        invertDrive = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //Command is constantly running, and can only be stopped manually
        return false;
    }
}
