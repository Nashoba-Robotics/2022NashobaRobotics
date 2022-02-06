package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GenericTalonSubsystem extends SubsystemBase{

    private TalonFX talon;

    private static GenericTalonSubsystem instance;

    public GenericTalonSubsystem(){
        talon = new TalonFX(0);
        configureMotor(talon);
    }

    public void setPosition(double pos){
        talon.set(ControlMode.Position, pos);
    }

    public static GenericTalonSubsystem getInstance(){
        if(instance == null){
            instance = new GenericTalonSubsystem();
        }
        return instance;
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

        motor.setNeutralMode(NeutralMode.Coast);

        motor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
    }
}
