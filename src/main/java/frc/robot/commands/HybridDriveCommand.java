package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/*
    Hybrid drive: Limelight controls turning, driver controls movement
*/
public class HybridDriveCommand extends CommandBase{

    LimelightSubsystem limelight;

    private boolean lastPressedbottom;
    private boolean bottomPressed;

    private JoystickButton joystickBottomRight;

    public HybridDriveCommand(){
        addRequirements(LimelightSubsystem.getInstance());
        addRequirements(AbstractDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // Set the Limelight pipeline
        LimelightSubsystem.getInstance().setPipeline(1);
        joystickBottomRight = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), 2);

        bottomPressed = joystickBottomRight.get();
        lastPressedbottom = joystickBottomRight.get();
    }

    @Override
    public void execute() {
        // Get the X position of the tracked object (-27 to +27)

        bottomPressed = joystickBottomRight.get();
        if(bottomPressed && lastPressedbottom != bottomPressed){
            CommandScheduler.getInstance().schedule(RobotContainer.joystickDriveCommand);
            CommandScheduler.getInstance().run();
            CommandScheduler.getInstance().cancel(RobotContainer.hybridDriveCommand);
        }
        lastPressedbottom = bottomPressed;

        double tx = LimelightSubsystem.getInstance().getTx();
        double turn = 0;

        if(LimelightSubsystem.getInstance().validTarget()) {
            // If there is a valid target
            if(Math.abs(tx) > 3) {
                // If the target's x position is outside of the deadzone,
                // turn an amount proportional to how far the target is left/right
                turn = tx/240;
            }
        }

        // Get the movement amount from the joysticks
        double move = JoystickSubsystem.getInstance().getLeftY();
        // Scale and shape the movement
        double moveScaled = JoystickProcessing.scaleJoystick(move, Constants.MOVEMENT_DEADZONE);
        double moveShaped = JoystickProcessing.shapeJoystick(moveScaled, Constants.MOVEMENT_SENSITIVITY);
        // Calculate motor values using arcade drive
        MotorValues vel = JoystickProcessing.arcadeDrive(new JoystickValues(moveShaped, turn));
        
        AbstractDriveSubsystem.getInstance().setSpeed(vel.left, vel.right);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        AbstractDriveSubsystem.getInstance().setRightMotorSpeed(0);
        AbstractDriveSubsystem.getInstance().setLeftMotorSpeed(0);
    }
}