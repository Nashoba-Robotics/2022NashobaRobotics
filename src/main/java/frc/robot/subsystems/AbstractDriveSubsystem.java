package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.StopCommand;

public abstract class AbstractDriveSubsystem extends SubsystemBase {
    private static AbstractDriveSubsystem instance;
    
    public enum DriveMode {
        VELOCITY, PERCENT;
    }

    public abstract void setSpeed(double left, double right);
    public abstract void setSpeed(double speed);
    public abstract void setDriveMode(DriveMode driveMode);
    public abstract double getLeftMotorVelocity();
    public abstract double getRightMotorVelocity();
    public abstract void setRawMotorVelocity(double left, double right);
    public abstract DriveMode getDriveMode();
    public abstract double getLeftMotorError();
    public abstract double getRightMotorError();
    public void setHDriveSpeed(double speed) {}

    public void initDefaultCommand(){
        setDefaultCommand(new StopCommand());
     }

    public static AbstractDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new Drive2019Subsystem();
        }
        return instance;
    }
}