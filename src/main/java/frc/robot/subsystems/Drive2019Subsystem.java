package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.Units;

// Subsystem for driving the robot
public class Drive2019Subsystem extends AbstractDriveSubsystem {
    public static final int VOLTAGE_COMPENSATION_LEVEL = 12;
    public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // find
    public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; // find this

    private DriveMode driveMode = DriveMode.VELOCITY;

    private TalonSRX leftMotor;
    private VictorSPX leftMotor2, leftMotor3;
    private TalonSRX rightMotor;
    private VictorSPX rightMotor2, rightMotor3;
    // private TalonSRX hDriveMotor;
    private CANSparkMax hDriveMotor;
    
    public Drive2019Subsystem() {
        leftMotor = new TalonSRX(Constants.LEFT_MOTOR_PORTS_2019[0]);
        leftMotor3 = new VictorSPX(Constants.LEFT_MOTOR_PORTS_2019[2]);
        leftMotor2 = new VictorSPX(Constants.LEFT_MOTOR_PORTS_2019[1]);
        rightMotor = new TalonSRX(Constants.RIGHT_MOTOR_PORTS_2019[0]);
        rightMotor2 = new VictorSPX(Constants.RIGHT_MOTOR_PORTS_2019[1]);
        rightMotor3 = new VictorSPX(Constants.RIGHT_MOTOR_PORTS_2019[2]);
        hDriveMotor = new CANSparkMax(Constants.HDRIVEPORT, MotorType.kBrushless);
        hDriveMotor.setInverted(true);
        leftMotor.setSensorPhase(true);
        leftMotor2.follow(leftMotor);
        leftMotor3.follow(leftMotor);
        rightMotor2.follow(rightMotor);
        rightMotor3.follow(rightMotor);

        //leftMasterSensor = new TalonFXSensorCollection(leftMotor);
        //rightMasterSensor = new TalonFXSensorCollection(rightMotor);

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

    private void configureMotor(TalonSRX motor) {
        motor.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		motor.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        //motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        //                                    Constants.PID_IDX, 
		//									Constants.TIMEOUT);
                                        
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
    }

    //takes input, speed, in form of percent (-1 through 1). Sets the speed of the right motor
    public void setRightMotorSpeed(double speed){
        TalonSRXControlMode controlMode = TalonSRXControlMode.Velocity;

        if(driveMode == DriveMode.VELOCITY){
            controlMode = TalonSRXControlMode.Velocity;
            speed = Units.percent2Velocity(speed);
        } else if(driveMode == DriveMode.PERCENT){
            controlMode = TalonSRXControlMode.PercentOutput;
        }
        
        //double aff = Constants.AFF * Math.signum(speed);
        double aff = 0;

        SmartDashboard.putNumber("Right input", speed);

        rightMotor.set(controlMode, speed, DemandType.ArbitraryFeedForward, aff);
    }

    //takes input, speed, in form of percent (-1 through 1). Sets the speed of the left motor
    public void setLeftMotorSpeed(double speed){
        TalonSRXControlMode controlMode = TalonSRXControlMode.Velocity;

        if(driveMode == DriveMode.VELOCITY){
            controlMode = TalonSRXControlMode.Velocity;
            speed = Units.percent2Velocity(speed);
        } else if(driveMode == DriveMode.PERCENT){
            controlMode = TalonSRXControlMode.PercentOutput;
        }

        //double aff = Constants.AFF * Math.signum(speed);
        double aff = 0;

        SmartDashboard.putNumber("Left input", speed);

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

    public void setHDriveSpeed(double speed) {
        hDriveMotor.set(speed);
        
    }

    public double getLeftMotorVelocity(){
        return 0;
        // return leftMasterSensor.getIntegratedSensorVelocity();
    }

    public double getRightMotorVelocity(){
        return 0;
        //return rightMasterSensor.getIntegratedSensorVelocity();
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
