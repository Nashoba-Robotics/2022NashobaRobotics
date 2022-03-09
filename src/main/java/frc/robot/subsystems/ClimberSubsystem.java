package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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

    // private DigitalInput lsLeft1;
    // private DigitalInput lsLeft2;
    // private DigitalInput lsRight1;
    // private DigitalInput lsRight2;

    private TalonFX[] motors;
    private boolean[] lastLSState;

    // private DigitalInput[] limitSwitches;

    private Joystick fixedClimbJoystick;
    private Joystick rotatingClimbJoystick;

    private TalonFX getMotor(ClimberMotor motor) {
        return motors[motor.ordinal()];
    }

    public boolean getLimitSwitch(ClimberMotor motor) {
         return getMotor(motor).getSensorCollection().isRevLimitSwitchClosed() == 1;
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

        fixedClimbJoystick = new Joystick(Constants.Climber.FIXED_MANUAL_CLIMB_JOYSTICK_PORT);
        rotatingClimbJoystick = new Joystick(Constants.Climber.ROTATING_MANUAL_CLIMB_JOYSTICK_PORT);

        motors = new TalonFX[] {
            motorLeft1, motorLeft2, motorLeftRotate, 
            motorRight1, motorRight2, motorRightRotate
        };

        lastLSState = new boolean[6];
        
        for(TalonFX motor: motors) {
            configureMotor(motor);
        }

        motorLeft1.setInverted(true);
        motorRight2.setInverted(true);

        motorLeft1.configForwardSoftLimitEnable(true);
        motorLeft1.configForwardSoftLimitThreshold(145500);
        motorLeft2.configForwardSoftLimitEnable(true);
        motorLeft2.configForwardSoftLimitThreshold(170000);
        motorRight1.configForwardSoftLimitEnable(true);
        motorRight1.configForwardSoftLimitThreshold(145500);
        motorRight2.configForwardSoftLimitEnable(true);
        motorRight2.configForwardSoftLimitThreshold(170000);

        motorLeftRotate.setNeutralMode(NeutralMode.Brake);
        motorRightRotate.setNeutralMode(NeutralMode.Brake);
       
        motorLeft1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motorLeft2.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       
        motorRight1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motorRight2.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        for(TalonFX motor1: new TalonFX[]{motorLeft1, motorRight1}) {
            motor1.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
            motor1.config_kP(0, Constants.Climber.KP_1, Constants.TIMEOUT);
            motor1.config_kI(0, Constants.Climber.KI_1, Constants.TIMEOUT);
            motor1.config_kD(0, Constants.Climber.KD_1, Constants.TIMEOUT);
        }

        for(TalonFX motor2: new TalonFX[]{motorLeft2, motorRight2}) {
            motor2.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
            motor2.config_kP(0, Constants.Climber.KP_2, Constants.TIMEOUT);
            motor2.config_kI(0, Constants.Climber.KI_2, Constants.TIMEOUT);
            motor2.config_kD(0, Constants.Climber.KD_2, Constants.TIMEOUT);
        }

        for(TalonFX rotate: new TalonFX[]{motorLeftRotate, motorRightRotate}) {
            rotate.config_kF(0, Constants.Climber.KF, Constants.TIMEOUT);
            rotate.config_kP(0, Constants.Climber.KP_ROTATE, Constants.TIMEOUT);
            rotate.config_kI(0, Constants.Climber.KI_ROTATE, Constants.TIMEOUT);
            rotate.config_kD(0, Constants.Climber.KD_ROTATE, Constants.TIMEOUT);
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

        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void setSpeed(ClimberMotor motor, double speed) {
        TalonFX talon = getMotor(motor);
        if(Math.abs(speed) <= 1) {
            talon.set(ControlMode.PercentOutput, speed);
        } else {
            talon.set(ControlMode.PercentOutput, 0);
        }
    }

    public void checkLimitSwitches() {
        // Zero on exit from limit switch
        for(int i = 0; i < motors.length; i++) {
            boolean ls = getLimitSwitch(ClimberMotor.values()[i]);
            boolean lastLs = lastLSState[i];

            // arm is leaving limit switch; zero position
            if(lastLs == true && ls == false) {
                getMotor(ClimberMotor.values()[i]).setSelectedSensorPosition(0);
            }
            
            lastLSState[i] = ls;
        }
    }

    public void deployClimb(){
        TalonFX leftClimber = getMotor(ClimberMotor.LEFT_1);
        TalonFX rightClimber = getMotor(ClimberMotor.RIGHT_1);
        leftClimber.configMotionAcceleration(Constants.Climber.DEPLOY_ACCELERATION);
        leftClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_CRUISE_VELOCITY);

        rightClimber.configMotionAcceleration(Constants.Climber.DEPLOY_ACCELERATION);
        rightClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_CRUISE_VELOCITY);

        leftClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_LEFT_POS);
        rightClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_RIGHT_POS);
    }

    public void undeployClimb(){
        TalonFX leftClimber = getMotor(ClimberMotor.LEFT_1);
        TalonFX rightClimber = getMotor(ClimberMotor.RIGHT_1);
        leftClimber.configMotionAcceleration(Constants.Climber.RETRACT_ACCELERATION);
        leftClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_CRUISE_VELOCITY);

        rightClimber.configMotionAcceleration(Constants.Climber.RETRACT_ACCELERATION);
        rightClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_CRUISE_VELOCITY);

        leftClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_POS);
        rightClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_POS);
    }

    public void mantainPositionRetract(ClimberMotor motor) {
        TalonFX talon = getMotor(motor);

        talon.set(ControlMode.Position, Constants.Climber.RETRACT_POS);
    }

    public void mantainPositionRetract() {
        mantainPositionRetract(ClimberMotor.LEFT_1);
        mantainPositionRetract(ClimberMotor.RIGHT_1);
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
        double MAX_CURRENT = 15;

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

    public double getFixedValue(){
        double rawY = fixedClimbJoystick.getY();
        double value = rawY * 0.15;
        return value;
    }

    public void manualFixedClimb(){
        getMotor(ClimberMotor.LEFT_1).set(ControlMode.PercentOutput, getFixedValue());
        getMotor(ClimberMotor.RIGHT_1).set(ControlMode.PercentOutput, getFixedValue());
    }

    public double getRotatingValue(){
        double rawX = rotatingClimbJoystick.getX();
        double value = rawX * 0.15;
        return value;
    }

    public void manualRotatingClimb(){
        getMotor(ClimberMotor.LEFT_1).set(ControlMode.PercentOutput, getRotatingValue());
        getMotor(ClimberMotor.RIGHT_1).set(ControlMode.PercentOutput, getRotatingValue());
    }
}
