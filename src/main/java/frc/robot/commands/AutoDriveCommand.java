package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.AccelerationControl;
import frc.robot.lib.JoystickProcessing;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.MotorValues;
import frc.robot.subsystems.AbstractDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/*
    Command for autonomous driving. Uses the Limelight to track objects and move towards
    them. If no object is found and spinning is enabled it will spin in place.
*/
public class AutoDriveCommand extends CommandBase{
    LimelightSubsystem limelight;
    private AccelerationControl accelerationControl;

    // +1 = clockwise, -1 = counterclockwise
    private int spinDirection;

    public AutoDriveCommand(){
        addRequirements(LimelightSubsystem.getInstance());
        addRequirements(AbstractDriveSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("move", 0);
        SmartDashboard.putNumber("distance auto", 0);
        SmartDashboard.putBoolean("target?", false);
        SmartDashboard.putNumber("spin?", 0);

        // Default to spinning clockwise
        spinDirection = 1;

        // Choose which Limelight pipeline is used
        LimelightSubsystem.getInstance().setPipeline(0);

        accelerationControl = new AccelerationControl(
            Constants.MAX_ACCEL, Constants.MAX_DECEL, 
            Constants.MAX_ACCEL_TURN, Constants.MAX_DECEL_TURN);
    }

    @Override
    public void execute() {
        // Get the X position of the tracked object (-27 to +27)
        double tx = LimelightSubsystem.getInstance().getTx();
        // Will be set in conditional later
        double turn = 0;
        double move = 0;
        // Should the robot spin in place if no object is found?
        boolean spin = SmartDashboard.getNumber("spin?", 0) == 1;

        if(LimelightSubsystem.getInstance().validTarget()) {
            // If a target is found

            // if(Math.abs(tx) > 5) {
            //     // If the x position of the target is outside of the deadzone,
            //     // calculate the turn speed based on how far left/right it is
            //     turn = -tx/170;
            // }

            double txPercent = -tx/27;
            if(Math.abs(txPercent) <= Constants.HYBRID_DRIVE_DEADZONE) turn = 0;
            else{
                turn = txPercent - Math.signum(txPercent) * Constants.HYBRID_DRIVE_DEADZONE;
                turn *= Constants.HYBRID_DRIVE_SENSITIVITY;
                turn = Math.pow(turn, 2);
                turn = Math.abs(turn);
                turn = Math.min(turn, 1);
                turn *= Math.signum(txPercent);
            }
            double ballDistance = LimelightSubsystem.getInstance().getDistanceBall();

            // Depending on the distance to the ball, there are three different behaviors:
            if (ballDistance > Constants.SPEED_THRESHOLD_AUTO) {
                // If the distance is greater than SPEED_THRESHOLD_AUTO, move at a constant speed (MOVE_SPEED_AUTO)
                move = Constants.MOVE_SPEED_AUTO;
            } else if(ballDistance > Constants.MIN_DISTANCE_AUTO) {
                // Otherwise, if the distance is greater than MIN_DISTANC_AUTO, move at a speed proportional to the distance
                // This formula is designed such that the distance-velocity graph is continuous
                move = ballDistance * Constants.MOVE_SPEED_AUTO / Constants.SPEED_THRESHOLD_AUTO;
            }
            // Otherwise, do not move
            
            // Set the spin direction based on whether the object is on the left or right half of the screen
            // This will ensure that the robot spins towards the object when it goes out of frame
            if(tx >= 0) {
                spinDirection = 1;
            } else {
                spinDirection = -1;
            }
        } else if(spin) {
            // If no object is tracked and spinning is enabled, spin
            turn = 0.05 * spinDirection;
        }

        // Use AccelerationControl to prevent tipping
        JoystickValues joystickValues = accelerationControl.next(new JoystickValues(move, turn));
        
        // Put diagnostics on Shuffleboard
        SmartDashboard.putNumber("move", move);
        SmartDashboard.putBoolean("target?", LimelightSubsystem.getInstance().validTarget());
        SmartDashboard.putNumber("distance auto", LimelightSubsystem.getInstance().getDistanceBall());
        
        // Calculate the motor velocities using arcade drive
        MotorValues vel = JoystickProcessing.arcadeDrive(joystickValues);
        
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
