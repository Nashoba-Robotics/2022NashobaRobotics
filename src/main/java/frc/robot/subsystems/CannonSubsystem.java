package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Units;

//public means accessible by other classes
//class means this is being defined as a class
//CannonSubsystem is what we are calling this class
//extends means it is using another class
//SubsystemBase allows us access to functions found in SubsystemBase
//{ indicates the start of the class
//(//) indicates it is a comment (not read by compiler) instead of code
//() is notation for signifying characters we are talking about inside the parentheses
public class CannonSubsystem extends SubsystemBase{
    private static CannonSubsystem singleton;
    public TalonFX topCannonMotor;
    public TalonFX bottomCannonMotor;

    private double speedChange;

    private Solenoid solenoid;

    public enum Angle {
        SIXTY, EIGHTY
    }

    public static CannonSubsystem getInstance(){
        if(singleton == null){
            singleton = new CannonSubsystem();
        }
        return singleton;   
    }

    public CannonSubsystem(){
        topCannonMotor = new TalonFX(Constants.Cannon.PORT_TOP);
        bottomCannonMotor = new TalonFX(Constants.Cannon.PORT_BOTTOM);
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Cannon.SOLENOID_PORT);
        topCannonMotor.setInverted(true);
        bottomCannonMotor.setInverted(true);
        configureMotor(topCannonMotor);
        configureMotor(bottomCannonMotor);

        speedChange = 0;
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
		motor.config_kF(Constants.SLOT_IDX, Constants.Cannon.KF, Constants.TIMEOUT);
		motor.config_kP(Constants.SLOT_IDX, Constants.Cannon.KP, Constants.TIMEOUT);
		motor.config_kI(Constants.SLOT_IDX, Constants.Cannon.KI, Constants.TIMEOUT);
        motor.config_kD(Constants.SLOT_IDX, Constants.Cannon.KD, Constants.TIMEOUT);

        motor.selectProfileSlot(Constants.SLOT_IDX, 0);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void changeSpeedChange(double delta){
        speedChange += delta;
    }

    public double getCurrentTop(){
        return topCannonMotor.getStatorCurrent();
    }

    public double getCurretBottom(){
        return bottomCannonMotor.getStatorCurrent();
    }

    //sets the speed of each individual motor
    public void set(double topSpeed, double bottomSpeed){
        topCannonMotor.set(ControlMode.Velocity, Units.percent2Velocity(topSpeed + speedChange));
        bottomCannonMotor.set(ControlMode.Velocity, Units.percent2Velocity(bottomSpeed + speedChange));
    }
    
    //sets the speed of both motors to same value
    public void set(double speed){
        set(speed, speed);
    }

    public void setProportional(double p){
        topCannonMotor.config_kP(Constants.SLOT_IDX, p, Constants.TIMEOUT);
        bottomCannonMotor.config_kP(Constants.SLOT_IDX, p, Constants.TIMEOUT);
    }

    public void setIntegral(double i){
        topCannonMotor.config_kI(Constants.SLOT_IDX, i, Constants.TIMEOUT);
        bottomCannonMotor.config_kI(Constants.SLOT_IDX, i, Constants.TIMEOUT);
    }

    public void setDerivative(double d){
        topCannonMotor.config_kD(Constants.SLOT_IDX, d, Constants.TIMEOUT);
        bottomCannonMotor.config_kD(Constants.SLOT_IDX, d, Constants.TIMEOUT);
    }

    public void setKF(double kF){
        topCannonMotor.config_kF(Constants.SLOT_IDX, kF, Constants.TIMEOUT);
        bottomCannonMotor.config_kF(Constants.SLOT_IDX, kF, Constants.TIMEOUT);
    }

    public void setAngle(Angle angle) {
        boolean on = angle == Angle.SIXTY;
        if(solenoid.get() != on) solenoid.set(on);
    }
}

