package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.lib.Units;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX motorUp1;
    private TalonFX motorUp2;
    private TalonFX motorDown;
    private TalonFX motorRotate;

    private static ClimberSubsystem singleton;
    public static ClimberSubsystem getInstance(){
        if(singleton == null){
            singleton = new ClimberSubsystem();
        }
        return singleton;   
    }

    private ClimberSubsystem() {
        motorUp1 = new TalonFX(Constants.Climber.PORT_UP1);
        motorUp2 = new TalonFX(Constants.Climber.PORT_UP2);
        motorDown = new TalonFX(Constants.Climber.PORT_DOWN);
        motorRotate = new TalonFX(Constants.Climber.PORT_ROTATE);

        configureMotor(motorUp1);
        configureMotor(motorUp2);
        configureMotor(motorDown);
        configureMotor(motorRotate);

		motorUp1.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
		motorUp1.config_kP(0, Constants.Climber.KP_UP, Constants.TIMEOUT);
		motorUp1.config_kI(0, Constants.Climber.KI_UP, Constants.TIMEOUT);
        motorUp1.config_kD(0, Constants.Climber.KD_UP, Constants.TIMEOUT);
        motorUp2.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
		motorUp2.config_kP(0, Constants.Climber.KP_UP, Constants.TIMEOUT);
		motorUp2.config_kI(0, Constants.Climber.KI_UP, Constants.TIMEOUT);
        motorUp2.config_kD(0, Constants.Climber.KD_UP, Constants.TIMEOUT);
        
		motorDown.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
		motorDown.config_kP(0, Constants.Climber.KP_DOWN, Constants.TIMEOUT);
		motorDown.config_kI(0, Constants.Climber.KI_DOWN, Constants.TIMEOUT);
        motorDown.config_kD(0, Constants.Climber.KD_DOWN, Constants.TIMEOUT);

        motorRotate.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
		motorRotate.config_kP(0, Constants.Climber.KP_ROTATE, Constants.TIMEOUT);
		motorRotate.config_kI(0, Constants.Climber.KI_ROTATE, Constants.TIMEOUT);
        motorRotate.config_kD(0, Constants.Climber.KD_ROTATE, Constants.TIMEOUT);
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

        motor.selectProfileSlot(Constants.SLOT_IDX, 0);

        motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms, 30);
        motor.configVelocityMeasurementWindow(8, 30);

        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void setPositionValue(double pos){
        motorRotate.setSelectedSensorPosition(pos, Constants.PID_IDX, Constants.TIMEOUT);
    }

    public void resetPositionValue(){
        motorRotate.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
    }

    public void setMotorRotate(double speed){
        motorRotate.set(ControlMode.Velocity, speed);
    }

    public void setMotorRotate(ControlMode mode, double value){
        motorRotate.set(mode, value);
    }

    public void setMotorDown(double speed){
        motorDown.set(ControlMode.Velocity, speed);
    }

    public void setMotorDown(ControlMode mode, double value){
        motorDown.set(mode, value);
    }

    public void setMotorUp(double speed){
        motorUp1.set(ControlMode.Velocity, Units.percent2Velocity(speed));
        motorUp2.set(ControlMode.Velocity, Units.percent2Velocity(speed));
    }

    public void setMotorUp(ControlMode mode, double value){
        motorUp1.set(mode, value);
        motorUp2.set(mode, value);
    }
}
