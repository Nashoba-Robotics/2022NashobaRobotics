package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.AccelerationControl;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.Units;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.AbstractDriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class JoystickDriveCommand extends CommandBase {

    private boolean arcadeDrive;    //toggles arcadeDrive
    private boolean buttonPressed;
    private boolean lastPressed;
    private JoystickButton joystickTrigger;

    // private double lastVelocity = 0;
    // private double lastMove = 0;
    // private double lastTurn = 0;

    private AccelerationControl accelerationControl;

    // mode: either DriveMode.VELOCITY for velocity
    // control or DriveMode.PERCENT for percent output control
    public JoystickDriveCommand() {
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(JoystickSubsystem.getInstance());
        joystickTrigger = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 1);
 
    }

    @Override
    public void initialize() {
        DriveSubsystem.getInstance().setSpeed(0, 0);
        arcadeDrive = true;
        buttonPressed = false;
        lastPressed = false;
        // lastVelocity = 0;
        // lastMillis = System.currentTimeMillis();

        accelerationControl = new AccelerationControl(
            Constants.MAX_ACCEL, Constants.MAX_DECEL, 
            Constants.MAX_ACCEL_TURN, Constants.MAX_DECEL_TURN);

        // lastMove = 0;
        // lastTurn = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    // TODO change from length 2 arrays to dedicated classes
    // TODO add options to enable/disable shuffleboard diagnostics, reorganize
    @Override
    public void execute() { //10/19/21 Joysticks output reverse values

        DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        if(joystickTrigger.get()) buttonPressed = true;
        else buttonPressed = false;

        if(buttonPressed && lastPressed != buttonPressed) arcadeDrive = !arcadeDrive;
        lastPressed = buttonPressed;

        // rightX: turning joystick
        double rightX = JoystickSubsystem.getInstance().getRightX();
        // leftY: movement joystick
        double leftY = JoystickSubsystem.getInstance().getLeftY();
        double leftX = JoystickSubsystem.getInstance().getLeftX();
        JoystickValues joystickValues = new JoystickValues(leftY, rightX);
        System.out.print(Units.roundTo(leftX, 3) + "," + Units.roundTo(leftY, 3) + " | ");

        joystickValues = JoystickProcessing.scaleJoysticks(joystickValues);
        joystickValues = JoystickProcessing.shapeJoysticks(joystickValues);

        System.out.print(Units.roundTo(joystickValues.movement, 3) + "," + Units.roundTo(joystickValues.turning, 3) + " | ");

        joystickValues = accelerationControl.next(joystickValues);

        // double moveChange = joystickValues.movement - lastMove;

        // double maxMove = Math.min(Math.abs(moveChange), getMaxChange(lastMove, joystickValues.movement, elapsed));
        // if(moveChange < 0) maxMove *= -1;
        // joystickValues.movement = lastMove + maxMove;
        // lastMove = joystickValues.movement;

        // double turnChange = joystickValues.turning - lastTurn;

        // double maxTurn = Math.min(Math.abs(turnChange), getMaxChangeTurn(lastTurn, joystickValues.turning, elapsed));
        // if(turnChange < 0) maxTurn *= -1;
        // joystickValues.turning = lastTurn + maxTurn;
        // lastTurn = joystickValues.turning;

        // System.out.print(Units.roundTo(joystickValues.movement, 3) + "," + Units.roundTo(joystickValues.turning, 3) + " | ");

        MotorValues motorValues;
        if(arcadeDrive){
            motorValues = JoystickProcessing.arcadeDrive(joystickValues);
        } 
        else{ 
            motorValues = JoystickProcessing.radiusDrive(joystickValues);
        }

        System.out.println(Units.roundTo(motorValues.left, 3) + "," + Units.roundTo(motorValues.right, 3));


        SmartDashboard.putBoolean("ArcadeDrive on?", arcadeDrive);

        // double maxVel = lastVelocity + Constants.MAX_ACCEL * elapsed;
        // double minVel = lastVelocity - Constants.MAX_DECEL * elapsed;

        // if(Math.abs(motorValues.left) > maxVel || Math.abs(motorValues.right) > maxVel) {
        //     double coeff = maxVel / Math.max(Math.abs(motorValues.left), Math.abs(motorValues.right));
        //     motorValues.left *= coeff;
        //     motorValues.right *= coeff;
        // }

        // if(Math.abs(motorValues.left) < minVel || Math.abs(motorValues.right) < minVel){
        //     double coeff = minVel / Math.min(Math.abs(motorValues.left), Math.abs(motorValues.right));
        //     motorValues.left *= coeff;
        //     motorValues.right *= coeff;
        //     System.out.println("working");
        // }
                
        DriveSubsystem.getInstance().setSpeed(motorValues.left, motorValues.right);
        DriveSubsystem.getInstance().setHDriveSpeed(leftX);
        
        SmartDashboard.putNumber("Joystick Left Y", leftY);
        SmartDashboard.putNumber("Joystick Right X", rightX);
        SmartDashboard.putNumber("Left percent", motorValues.left);
        SmartDashboard.putNumber("Right percent", motorValues.right);
        //Puts the velocity that the motor controller is reading to SmartDashboard
        SmartDashboard.putNumber("Left Motor Real Velocity", DriveSubsystem.getInstance().getLeftMotorVelocity());
        SmartDashboard.putNumber("Right Motor Real Velocity", DriveSubsystem.getInstance().getRightMotorVelocity());

        // SmartDashboard.putNumber("Left Motor Real Velocity", PercentOutputDriveSubsystem.getInstance().getLeftMotorVelocity());
        // SmartDashboard.putNumber("Right Motor Real Velocity", PercentOutputDriveSubsystem.getInstance().getRightMotorVelocity());

        SmartDashboard.putNumber("Left Motor Current", DriveSubsystem.getInstance().getLeftMotorCurrent());
        SmartDashboard.putNumber("Right Motor Current", DriveSubsystem.getInstance().getRightMotorCurrent());
    
        // lastVelocity = Math.max(Math.abs(motorValues.left), Math.abs(motorValues.right));
        // lastMillis = System.currentTimeMillis();
    }

    // private double getMaxChange(double lastValue, double newValue, long elapsed) {
    //     if( (lastValue < 0 && newValue > 0)
    //     || (newValue < 0 && lastValue > 0)
    //      || Math.abs(newValue) < Math.abs(lastValue) ) {
    //         return Constants.MAX_DECEL * elapsed;
    //     }
    //     return Constants.MAX_ACCEL * elapsed;
    // }

    // private double getMaxChangeTurn(double lastValue, double newValue, long elapsed) {
    //     if( (lastValue < 0 && newValue > 0)
    //     || (newValue < 0 && lastValue > 0)
    //      || Math.abs(newValue) < Math.abs(lastValue) ) {
    //         return Constants.MAX_DECEL_TURN * elapsed;
    //     }
    //     return Constants.MAX_ACCEL_TURN * elapsed;
    // }

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
