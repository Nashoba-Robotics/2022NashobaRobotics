package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import edu.wpi.first.util.sendable.SendableRegistry;

import frc.robot.Constants;
import frc.robot.lib.Units;

// Subsystem for driving the robot
public class DriveSubsystem extends AbstractDriveSubsystem {
    //public static final double KF = 0.0475;
    public static final int VOLTAGE_COMPENSATION_LEVEL = 12;
    public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // find
    public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; // find this

    private DriveMode driveMode = DriveMode.VELOCITY;

    private TalonFX leftMotor, leftMotor2, leftMotor3;
    private TalonFXSensorCollection leftMasterSensor;
    private TalonFX rightMotor, rightMotor2, rightMotor3;
    private TalonFXSensorCollection rightMasterSensor;

    
    public DriveSubsystem() {
        leftMotor = new TalonFX(Constants.LEFT_MOTOR_PORTS[0]);
        leftMotor3 = new TalonFX(Constants.LEFT_MOTOR_PORTS[2]);
        leftMotor2 = new TalonFX(Constants.LEFT_MOTOR_PORTS[1]);
        rightMotor = new TalonFX(Constants.RIGHT_MOTOR_PORTS[0]);
        rightMotor2 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[1]);
        rightMotor3 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[2]);
        leftMotor2.follow(leftMotor);
        leftMotor3.follow(leftMotor);
        rightMotor2.follow(rightMotor);
        rightMotor3.follow(rightMotor);

        leftMasterSensor = new TalonFXSensorCollection(leftMotor);
        rightMasterSensor = new TalonFXSensorCollection(rightMotor);

        configureMotor(rightMotor);
        configureMotor(leftMotor);
        
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
        rightMotor2.setInverted(true);
        leftMotor2.setInverted(false);
        rightMotor3.setInverted(true);
        leftMotor3.setInverted(false);

        // Set the name of the subsystem in smart dashboard
        SendableRegistry.setName(this, "Drive");
    }

    private void configureMotor(TalonFX motor) {
        motor.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		motor.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.PID_IDX, 
											Constants.TIMEOUT);
                                                
		/* Config the peak and nominal outputs */
		motor.configNominalOutputForward(0, Constants.TIMEOUT);
		motor.configNominalOutputReverse(0, Constants.TIMEOUT);
		motor.configPeakOutputForward(1, Constants.TIMEOUT);
		motor.configPeakOutputReverse(-1, Constants.TIMEOUT);

		/* Config the Velocity closed loop gains in slot0 */
		motor.config_kF(Constants.SLOT_IDX, Constants.KF, Constants.TIMEOUT);
		motor.config_kP(Constants.SLOT_IDX, Constants.KP, Constants.TIMEOUT);
		motor.config_kI(Constants.SLOT_IDX, Constants.KI, Constants.TIMEOUT);
        motor.config_kD(Constants.SLOT_IDX, Constants.KD, Constants.TIMEOUT);

        motor.selectProfileSlot(Constants.SLOT_IDX, 0);

        motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms, 30);
        motor.configVelocityMeasurementWindow(8, 30);

        
    }

    public void setProportional(double p){
        rightMotor.config_kP(Constants.SLOT_IDX, p, Constants.TIMEOUT);
        leftMotor.config_kP(Constants.SLOT_IDX, p, Constants.TIMEOUT);
    }

    public void setIntegral(double i){
        rightMotor.config_kI(Constants.SLOT_IDX, i, Constants.TIMEOUT);
        leftMotor.config_kI(Constants.SLOT_IDX, i, Constants.TIMEOUT);
    }

    public void setDerivative(double d){
        rightMotor.config_kD(Constants.SLOT_IDX, d, Constants.TIMEOUT);
        leftMotor.config_kD(Constants.SLOT_IDX, d, Constants.TIMEOUT);
    }

    //takes input, speed, in form of percent (-1 through 1). Sets the speed of the right motor
    public void setRightMotorSpeed(double speed){
        TalonFXControlMode controlMode = TalonFXControlMode.Velocity;

        if(driveMode == DriveMode.VELOCITY){
            controlMode = TalonFXControlMode.Velocity;
            speed = Units.percent2Velocity(speed);
        } else if(driveMode == DriveMode.PERCENT){
            controlMode = TalonFXControlMode.PercentOutput;
        }
        
        double aff = Constants.AFF * Math.signum(speed);

        rightMotor.set(controlMode, speed, DemandType.ArbitraryFeedForward, aff);
    }

    //takes input, speed, in form of percent (-1 through 1). Sets the speed of the left motor
    public void setLeftMotorSpeed(double speed){
        TalonFXControlMode controlMode = TalonFXControlMode.Velocity;

        if(driveMode == DriveMode.VELOCITY){
            controlMode = TalonFXControlMode.Velocity;
            speed = Units.percent2Velocity(speed);
        } else if(driveMode == DriveMode.PERCENT){
            controlMode = TalonFXControlMode.PercentOutput;
        }

        double aff = Constants.AFF * Math.signum(speed);

        leftMotor.set(controlMode, speed, DemandType.ArbitraryFeedForward, aff);
    }

    // Set the speed of both the left and right side
    public void setSpeed(double speed) {
        setSpeed(speed, speed);
    }

    // Set the left and right sides separately
    public void setSpeed(double left, double right) {
        setLeftMotorSpeed(left);
        setRightMotorSpeed(right);
    }

    public void setRawMotorVelocity(double left, double right){
        //left = Units.percent2Velocity(left);
        //right = Units.percent2Velocity(right);
        leftMotor.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, Constants.AFF);
        rightMotor.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, Constants.AFF);
    }

    public void setRawPercent(double left, double right){
        leftMotor.set(ControlMode.PercentOutput, left, DemandType.ArbitraryFeedForward, Constants.AFF);
        rightMotor.set(ControlMode.PercentOutput, right, DemandType.ArbitraryFeedForward, Constants.AFF);
    }

    public double getLeftMotorVelocity(){
        return leftMasterSensor.getIntegratedSensorVelocity();
    }

    public double getRightMotorVelocity(){
        return rightMasterSensor.getIntegratedSensorVelocity();
    }

    public double getLeftMotorError() {
        return leftMotor.getClosedLoopError();
    }

    public double getRightMotorError() {
        return rightMotor.getClosedLoopError();
    }

    // Input: DriveMode enum. Sets drive mode the robot will be in (VELOCITY/PERCENT)
    public void setDriveMode(DriveMode mode){
        driveMode = mode;
    }

    public DriveMode getDriveMode(){
        return driveMode;
    }

}
