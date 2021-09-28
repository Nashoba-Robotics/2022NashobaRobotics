package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Checks if everything is connect and works properly
public class DiagnosticCommand extends CommandBase{
    private TalonFX[] motors;
    private int[] motorPorts;
    private Timer timer;
    int totalMotorLength;
    private final double motorTestSpeed = 0.1;

    public DiagnosticCommand(){
        timer = new Timer();
        int leftMotorLength = Constants.LEFT_MOTOR_PORTS.length;
        int rightMotorLength = Constants.RIGHT_MOTOR_PORTS.length;
        int miscMotorLength = Constants.MISC_MOTOR_PORTS.length;
        totalMotorLength = leftMotorLength + rightMotorLength + miscMotorLength;

        motors = new TalonFX[totalMotorLength];
        motorPorts = new int[totalMotorLength];

        //Initializes all of the motors from Constants in the motors array
        int i;
        for(i = 0; i < leftMotorLength; i++){
            motors[i] = new TalonFX(Constants.LEFT_MOTOR_PORTS[i]);
            motorPorts[i] = Constants.LEFT_MOTOR_PORTS[i];
        }
        for(; i < leftMotorLength + rightMotorLength; i++){
            motors[i] = new TalonFX(Constants.RIGHT_MOTOR_PORTS[i-leftMotorLength]);
            motorPorts[i] = Constants.RIGHT_MOTOR_PORTS[i-leftMotorLength];
        }
        for(; i < leftMotorLength + rightMotorLength + miscMotorLength; i++){
            motors[i] = new TalonFX(Constants.MISC_MOTOR_PORTS[i-leftMotorLength-rightMotorLength]);
            motorPorts[i] = Constants.MISC_MOTOR_PORTS[i-leftMotorLength-rightMotorLength];
        }
    }
    
    //Only run everything once, so don't need any other method
    @Override
    public void initialize() {
        /*
        Motors:
        Reset Timer
        Turn motor on
        After 5 seconds, check if the motor position is different from the initial position
        */
        TalonFXSensorCollection sensor = new TalonFXSensorCollection(motors[0]);
        double initialPosition = sensor.getIntegratedSensorPosition();
        motors[0].set(ControlMode.PercentOutput, motorTestSpeed);
        timer.start();
        int i = 0;
        while(i < totalMotorLength){
            SmartDashboard.putNumber("Time", timer.get());
            SmartDashboard.putString("Motor " + motorPorts[i], "Pending...");
            //IMPORTANT: Some motors may have a limited axis of rotation
            if(timer.get() >= 5){    //Timer in seconds
                //TODO: Condition MIGHT be too strict, change sensitivity
                if(sensor.getIntegratedSensorPosition() != initialPosition){
                    SmartDashboard.putString("Motor " + motorPorts[i], "Success");
                }
                else{
                    SmartDashboard.putString("Motor " + motorPorts[i], "Failure");
                }
                motors[i].set(ControlMode.PercentOutput, 0);
                i++;
                timer.reset();
                sensor = new TalonFXSensorCollection(motors[i]);
                initialPosition = sensor.getIntegratedSensorPosition();
                motors[i].set(ControlMode.PercentOutput, motorTestSpeed);
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
    
}