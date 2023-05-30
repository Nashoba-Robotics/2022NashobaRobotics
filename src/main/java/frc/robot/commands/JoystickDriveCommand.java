package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
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

    // The left and right joystick triggers
    private JoystickButton joystickTriggerLeft;


    public JoystickDriveCommand() {
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(JoystickSubsystem.getInstance());
        joystickTriggerLeft = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 1);
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

        //SmartDashboard.putBoolean("Drive Running", true);
    }

    @Override
    public void execute() {
        // Joystick drive uses velocity mode
        DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        
        // Toggle arcade drive when the left trigger is pressed
        //Temporarily switch left trigger to invert drive
        buttonPressedLeft = joystickTriggerLeft.get();
        if(buttonPressedLeft && lastPressedLeft != buttonPressedLeft){
            invertDrive = !invertDrive;
        } 
        lastPressedLeft = buttonPressedLeft;

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

        //Switches between Arcade Drive and Radius Drive
        //motorValues are move/turn instead of left/right
        MotorValues motorValues;
        if(arcadeDrive){
            motorValues = JoystickProcessing.arcadeDrive(joystickValues, invertDrive);
        } 
        else{ 
            motorValues = JoystickProcessing.radiusDrive(joystickValues);
        }

        // SmartDashboard.putBoolean("ArcadeDrive on?", arcadeDrive);
        // SmartDashboard.putBoolean("InvertedDrive?", invertDrive);
                
        //Sets the speed
        if(!invertDrive) DriveSubsystem.getInstance().setSpeed(motorValues.left * 0.2, motorValues.right * 0.2);
        else DriveSubsystem.getInstance().setSpeed(-motorValues.left * 0.2, -motorValues.right * 0.2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //When this command ends, set everything to 0 and gets out of Brake Mode
        DriveSubsystem.getInstance().setDriveMode(DriveMode.PERCENT);
        DriveSubsystem.getInstance().setSpeed(0, 0);
        DriveSubsystem.getInstance().setBrakeMode(false);
        invertDrive = false;
        // SmartDashboard.putBoolean("Drive Running", false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //Command is constantly running, and can only be stopped manually
        return false;
    }
}
