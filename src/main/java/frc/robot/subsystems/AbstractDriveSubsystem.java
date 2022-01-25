package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.StopCommand;
import frc.robot.subsystems.DriveSubsystem;

public abstract class AbstractDriveSubsystem extends SubsystemBase {
    private static AbstractDriveSubsystem instance;
    
    public enum DriveMode {
        VELOCITY, PERCENT;
    }

    public abstract void setProportional(double p);
    public abstract void setIntegral(double i);
    public abstract void setDerivative(double d);

    public abstract void changeBrakeMode();
    public abstract boolean getBrakeMode();
    public abstract void setBrakeMode(boolean brakeMode);

    //implement if we want to be able to set specific motors
    // public abstract void setProportional(BaseTalon motor, double p);
    // public abstract void setIntegral(BaseTalon motor, double i);
    // public abstract void setDerivative(BaseTalon motor, double d);
    
    public abstract void setSpeed(double left, double right);
    public abstract void setSpeed(double speed);
    public abstract void setRawPercent(double left, double right);
    public abstract void setRightMotorSpeed(double speed);
    public abstract void setLeftMotorSpeed(double speed);
    public abstract void setDriveMode(DriveMode driveMode);
    public abstract double getLeftMotorVelocity();
    public abstract double getRightMotorVelocity();
    public abstract void setRawMotorVelocity(double left, double right);
    public abstract DriveMode getDriveMode();
    public abstract double getLeftMotorError();
    public abstract double getRightMotorError();
    public abstract double getLeftMotorCurrent();
    public abstract double getRightMotorCurrent();
    public void setHDriveSpeed(double speed) {}

    public void initDefaultCommand(){
        setDefaultCommand(new StopCommand());
     }

    public static AbstractDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new DriveSubsystem();
        }
        return instance;
    }
}