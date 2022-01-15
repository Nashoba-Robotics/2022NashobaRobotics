package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Units;

public class CannonSubsystem extends SubsystemBase{
    private static CannonSubsystem singleton;
    public TalonFX topCannonMotor;
    public TalonFX bottomCannonMotor;
    public static CannonSubsystem getInstance(){
        if(singleton == null){
            singleton = new CannonSubsystem();
        }
        return singleton;   
    }

    public CannonSubsystem(){
        topCannonMotor = new TalonFX(0);
        bottomCannonMotor = new TalonFX(7);
        //bottomCannonMotor.setInverted(true);
        configureMotor(topCannonMotor);
        configureMotor(bottomCannonMotor);
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
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void shoot(double topSpeed, double bottomSpeed){
        topCannonMotor.set(ControlMode.Velocity, Units.percent2Velocity(topSpeed));
        bottomCannonMotor.set(ControlMode.Velocity, Units.percent2Velocity(bottomSpeed));
    }
    public void shoot(double speeeeeeed){
        shoot(speeeeeeed, speeeeeeed);
    }
}

