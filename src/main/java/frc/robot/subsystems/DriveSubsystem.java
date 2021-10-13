package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import frc.robot.Constants;
import frc.robot.commands.StopCommand;

// Subsystem for driving the robot
public class DriveSubsystem extends SubsystemBase {
    public static final double KF = 0.0475;
    public static final int VOLTAGE_COMPENSATION_LEVEL = 12;
    public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // find
    public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; // find this

    // The singleton instance; generated on the first call of getInstance()
    private static DriveSubsystem instance;

    private TalonFX leftMotor, leftMotor2, leftMotor3;
    private TalonFXSensorCollection leftMasterSensor;
    private TalonFX rightMotor, rightMotor2, rightMotor3;
    private TalonFXSensorCollection rightMasterSensor;

    private DriveSubsystem() {
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
        
        rightMotor.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		rightMotor.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.PID_LOOP_IDX, 
											Constants.TIMEOUT);
											

		/* Config the peak and nominal outputs */
		rightMotor.configNominalOutputForward(0, Constants.TIMEOUT);
		rightMotor.configNominalOutputReverse(0, Constants.TIMEOUT);
		rightMotor.configPeakOutputForward(1, Constants.TIMEOUT);
		rightMotor.configPeakOutputReverse(-1, Constants.TIMEOUT);

		/* Config the Velocity closed loop gains in slot0 */
		rightMotor.config_kF(Constants.PID_LOOP_IDX, Constants.KF, Constants.TIMEOUT);
		rightMotor.config_kP(Constants.PID_LOOP_IDX, Constants.KP, Constants.TIMEOUT);
		rightMotor.config_kI(Constants.PID_LOOP_IDX, Constants.KI, Constants.TIMEOUT);
        rightMotor.config_kD(Constants.PID_LOOP_IDX, Constants.KD, Constants.TIMEOUT);
        
        leftMotor.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		leftMotor.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.PID_LOOP_IDX, 
											Constants.TIMEOUT);
											

		/* Config the peak and nominal outputs */
		leftMotor.configNominalOutputForward(0, Constants.TIMEOUT);
		leftMotor.configNominalOutputReverse(0, Constants.TIMEOUT);
		leftMotor.configPeakOutputForward(1, Constants.TIMEOUT);
		leftMotor.configPeakOutputReverse(-1, Constants.TIMEOUT);

		/* Config the Velocity closed loop gains in slot0 */
		leftMotor.config_kF(Constants.PID_LOOP_IDX, Constants.KF, Constants.TIMEOUT);
		leftMotor.config_kP(Constants.PID_LOOP_IDX, Constants.KP, Constants.TIMEOUT);
		leftMotor.config_kI(Constants.PID_LOOP_IDX, Constants.KI, Constants.TIMEOUT);
		leftMotor.config_kD(Constants.PID_LOOP_IDX, Constants.KD, Constants.TIMEOUT);

        // Set the name of the subsystem in smart dashboard
        SendableRegistry.setName(this, "Drive");
    }
    public void initDefaultCommand(){
       setDefaultCommand(new StopCommand());
    }
    public static DriveSubsystem getInstance() {
        if(instance == null) {
            instance = new DriveSubsystem();
        }
        return instance;
    }

    // Set the speed of both the left and right side
    public void setSpeed(double speed, ControlMode mode) {
        setSpeed(speed, speed, mode);
    }

    // Set the left and right sides separately
    public void setSpeed(double left, double right, ControlMode mode) {
        if(mode == ControlMode.PercentOutput) {
            leftMotor.set(mode, left, DemandType.ArbitraryFeedForward, Constants.AFF);
            rightMotor.set(mode, right, DemandType.ArbitraryFeedForward, Constants.AFF);
            //leftMotor.set(mode, left);
            //rightMotor.set(mode, right);
        } else {
            leftMotor.set(mode, left, DemandType.ArbitraryFeedForward, Constants.AFF);
            rightMotor.set(mode, right, DemandType.ArbitraryFeedForward, Constants.AFF);
            //leftMotor.set(mode, left);
            //rightMotor.set(mode, right);
        }
    }

    public double getLeftMotorVelocity(){
        return leftMasterSensor.getIntegratedSensorVelocity();
    }

    public double getRightMotorVelocity(){
        return rightMasterSensor.getIntegratedSensorVelocity();

    }

}
