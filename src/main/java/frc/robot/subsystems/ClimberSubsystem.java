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
    public enum ClimberMotor {
        LEFT_1, LEFT_2, LEFT_ROTATE, 
        RIGHT_1, RIGHT_2, RIGHT_ROTATE
    }
    private TalonFX motorLeft1;
    private TalonFX motorLeft2;
    private TalonFX motorLeftRotate;
    private TalonFX motorRight1;
    private TalonFX motorRight2;
    private TalonFX motorRightRotate;

    private DigitalInput lsLeft1;
    private DigitalInput lsLeft2;
    private DigitalInput lsRight1;
    private DigitalInput lsRight2;

    private TalonFX[] motors = {
        motorLeft1, motorLeft2, motorLeftRotate, 
        motorRight1, motorRight2, motorRightRotate
    };

    private DigitalInput[] limitSwitches = {
        lsLeft1, lsRight1, null,
        lsLeft2, lsRight2, null
    }

    private TalonFX getMotor(ClimberMotor motor) {
        return motors[motor.ordinal()];
    }

    private DigitalInput getLimitSwitch(ClimberMotor motor) {
        return limitSwitches[motor.ordinal()];
    }

    private static ClimberSubsystem singleton;

    public static ClimberSubsystem getInstance(){
        if(singleton == null){
            singleton = new ClimberSubsystem();
        }
        return singleton;   
    }

    private ClimberSubsystem() {
        motorLeft1 = new TalonFX(Constants.Climber.PORT_LEFT_1);
        motorLeft2 = new TalonFX(Constants.Climber.PORT_LEFT_2);
        motorLeftRotate = new TalonFX(Constants.Climber.PORT_LEFT_ROTATE);
        motorRight1 = new TalonFX(Constants.Climber.PORT_RIGHT_1);
        motorRight2 = new TalonFX(Constants.Climber.PORT_RIGHT_2);
        motorRightRotate = new TalonFX(Constants.Climber.PORT_RIGHT_ROTATE);

        lsLeft1 = new DigitalInput(Constants.Climber.DIO_LS_LEFT_1);
        lsLeft2 = new DigitalInput(Constants.Climber.DIO_LS_LEFT_2);
        lsRight1 = new DigitalInput(Constants.Climber.DIO_LS_RIGHT_1);
        lsRight2 = new DigitalInput(Constants.Climber.DIO_LS_RIGHT_2);

        // TODO TODO TODO
        // Limit switch notes:
        // to connect the motor directly to the limit switch, use:
        // motor.config[Forward|Reverse]LimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal.Normally[Open|Closed], id, 0);
        // this requires a CANifier: https://store.ctr-electronics.com/canifier/
        // i do not believe we have four of these, so we will have to do it using code

        for(TalonFX motor: motors) {
            configureMotor(motor);
        }

        for(TalonFX motor1: new TalonFX[]{motorLeft1, motorRight1}) {
            motor1.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
            motor1.config_kP(0, Constants.Climber.KP_1, Constants.TIMEOUT);
            motor1.config_kI(0, Constants.Climber.KI_1, Constants.TIMEOUT);
            motor1.config_kD(0, Constants.Climber.KD_1, Constants.TIMEOUT);
            motor1.setSelectedSensorPosition(0);
        }

        for(TalonFX motor2: new TalonFX[]{motorLeft2, motorRight2}) {
            motor2.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
            motor2.config_kP(0, Constants.Climber.KP_2, Constants.TIMEOUT);
            motor2.config_kI(0, Constants.Climber.KI_2, Constants.TIMEOUT);
            motor2.config_kD(0, Constants.Climber.KD_2, Constants.TIMEOUT);
            motor2.setSelectedSensorPosition(0);
        }

        for(TalonFX rotate: new TalonFX[]{motorLeftRotate, motorRightRotate}) {
            rotate.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
            rotate.config_kP(0, Constants.Climber.KP_ROTATE, Constants.TIMEOUT);
            rotate.config_kI(0, Constants.Climber.KI_ROTATE, Constants.TIMEOUT);
            rotate.config_kD(0, Constants.Climber.KD_ROTATE, Constants.TIMEOUT);
            rotate.setSelectedSensorPosition(0);
        }
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

    public void setSpeed(ClimberMotor motor, double speed) {
        TalonFX talon = getMotor(motor);
        DigitalInput ls = getLimitSwitch(motor);
        // TODO limit switch might be inverted, add ! possibly
        // TODO one motion direction should be allowed here
        if(ls != null && ls.get())
            talon.set(ControlMode.PercentOutput, 0);
            // TODO reset talon position here?
            return
        }
        if(Math.abs(speed) < 0.05) {
            talon.set(ControlMode.PercentOutput, speed);
        } else {
            talon.set(ControlMode.PercentOutput, 0);
        }
    }

    private void checkLimitSwitches() {
        for(int i = 0; i < motors.length; i++) {
            TalonFX talon = motors[i];
            DigitalInput ls = limitSwitches[i];
            // TODO limit switch might be inverted, add ! possibly
            if(ls != null && ls.get()) {
                talon.set(ControlMode.PercentOutput, 0);
                // TODO reset talon position here?
            }
        }
    }

    public double getPosition(ClimberMotor motor) {
        return getMotor(motor).getSelectedSensorPosition();
    }

    public double getStatorCurrent(ClimberMotor motor) {
        return getMotor(motor).getStatorCurrent();
    }

    public double getSupplyCurrent(ClimberMotor motor) {
        return getMotor(motor).getSupplyCurrent();
    }

    public void stop() {
        for(TalonFX motor: motors) {
            motor.set(ControlMode.PercentOutput, 0);
        }
    }

    public boolean isCurrentBad() {
        double MAX_CURRENT = 10;

        double currentLeft1 = Math.abs(motorLeft1.getStatorCurrent());
        double currentLeft2 = Math.abs(motorLeft2.getStatorCurrent());
        double currentLeftRotate = Math.abs(motorLeftRotate.getStatorCurrent());
        double currentRight1 = Math.abs(motorRight1.getStatorCurrent());
        double currentRight2 = Math.abs(motorRight2.getStatorCurrent());
        double currentRightRotate = Math.abs(motorRightRotate.getStatorCurrent());

        return currentLeft1 > MAX_CURRENT
        || currentLeft2 > MAX_CURRENT
        || currentLeftRotate > MAX_CURRENT
        || currentRight1 > MAX_CURRENT
        || currentRight2 > MAX_CURRENT
        || currentRightRotate > MAX_CURRENT;
    }
}