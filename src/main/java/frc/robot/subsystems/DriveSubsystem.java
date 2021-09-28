package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import frc.robot.Constants;

// Subsystem for driving the robot
public class DriveSubsystem extends SubsystemBase {
    // The singleton instance; generated on the first call of getInstance()
    private static DriveSubsystem instance;

    private TalonFX leftMotor1, leftMotor2, leftMotor3;
    private TalonFX rightMotor1, rightMotor2, rightMotor3;

    private DriveSubsystem() {
        leftMotor1 = new TalonFX(Constants.LEFT_MOTOR_PORTS[0]);
        leftMotor3 = new TalonFX(Constants.LEFT_MOTOR_PORTS[2]);
        leftMotor2 = new TalonFX(Constants.LEFT_MOTOR_PORTS[1]);
        rightMotor1 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[0]);
        rightMotor2 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[1]);
        rightMotor3 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[2]);
        leftMotor2.follow(leftMotor1);
        leftMotor3.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
        rightMotor3.follow(rightMotor1);
        // Set the name of the subsystem in smart dashboard
        SendableRegistry.setName(this, "Drive");
    }

    public static DriveSubsystem getInstance() {
        if(instance == null) {
            instance = new DriveSubsystem();
        }
        return instance;
    }

    // Set the speed of both the left and right side
    // speed ranges from -1 to 1
    public void setSpeed(double speed) {
        setSpeed(speed, speed);
    }

    // Set the left and right sides separately
    // left and right range from -1 to 1
    public void setSpeed(double left, double right) {
        leftMotor1.set(ControlMode.PercentOutput, -left);
        rightMotor1.set(ControlMode.PercentOutput, right);
    }
}
