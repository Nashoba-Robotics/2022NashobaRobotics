package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

/*
    Hybrid drive: Limelight controls turning, driver controls movement
*/
public class HybridDriveCommand extends CommandBase{

    private boolean lastPressedbottom;
    private boolean bottomPressed;

    private JoystickButton joystickBottomRight;

    public HybridDriveCommand(){
        addRequirements(LimelightSubsystem.getInstance());
        addRequirements(PhotonVisionSubsystem.getInstance());
        addRequirements(DriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // Set the Limelight pipeline
        //LimelightSubsystem.getInstance().setPipeline(2);
        joystickBottomRight = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), 2);

        bottomPressed = joystickBottomRight.get();
        lastPressedbottom = joystickBottomRight.get();
    }

    @Override
    public void execute() {
        // Get the X position of the tracked object (-27 to +27)
        PhotonVisionSubsystem.getInstance().update();
        bottomPressed = joystickBottomRight.get();
        if(bottomPressed && lastPressedbottom != bottomPressed){
            CommandScheduler.getInstance().schedule(RobotContainer.joystickDriveCommand);
            CommandScheduler.getInstance().run();
            CommandScheduler.getInstance().cancel(RobotContainer.hybridDriveCommand);
        }
        lastPressedbottom = bottomPressed;

        double tx = -PhotonVisionSubsystem.getInstance().getTx();
        SmartDashboard.putNumber("HybridTX", tx);
        double turn = 0;

        if(PhotonVisionSubsystem.getInstance().validTarget()) {
            // If there is a valid target
            // if(Math.abs(tx) > 3) {
            //     // If the target's x position is outside of the deadzone,
            //     // turn an amount proportional to how far the target is left/right
            //     turn = tx/240;
            // }

            double txPercent = tx/27;
            if(Math.abs(txPercent) <= Constants.HYBRID_DRIVE_DEADZONE) turn = 0;
            else{
                turn = txPercent - Math.signum(txPercent) * Constants.HYBRID_DRIVE_DEADZONE;
                turn *= Constants.HYBRID_DRIVE_SENSITIVITY;
                turn = Math.pow(turn, 2);
                turn = Math.abs(turn);
                turn = Math.min(turn, 1);
                turn *= Math.signum(txPercent);
            } //turn = Math.signum(txPercent) * Math.min(Math.abs(Math.pow((Constants.HYBRID_DRIVE_SENSITIVITY) * (txPercent - Math.signum(txPercent)*Constants.HYBRID_DRIVE_DEADZONE), 5)), 1);
        }

        SmartDashboard.putNumber("Hybrid Turn", turn);

        // Get the movement amount from the joysticks
        double move = JoystickSubsystem.getInstance().getLeftY();
        // Scale and shape the movement
        double moveScaled = JoystickProcessing.scaleJoystick(move, Constants.MOVEMENT_DEADZONE);
        double moveShaped = JoystickProcessing.shapeJoystick(moveScaled, Constants.MOVEMENT_SENSITIVITY);
        // Calculate motor values using arcade drive
        MotorValues vel = JoystickProcessing.arcadeDrive(new JoystickValues(moveShaped, turn));
        
        DriveSubsystem.getInstance().setSpeed(vel.left, vel.right);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        DriveSubsystem.getInstance().setRightMotorSpeed(0);
        DriveSubsystem.getInstance().setLeftMotorSpeed(0);
    }
}