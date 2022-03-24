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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.lib.Units;

public class ClimberSubsystem extends SubsystemBase {
    public enum ClimberMotor {
        LEFT_CLIMBER,
        RIGHT_CLIMBER
    }

    private TalonFX leftClimber;
    // private TalonFX leftPusher;
    private TalonFX rightClimber;
    // private TalonFX rightPusher;

    private TalonFX[] motors;
    private boolean[] lastLSState;

    // private Joystick fixedClimbJoystick;
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
        leftClimber = new TalonFX(Constants.Climber.LEFT_CLIMBER_PORT);
        // leftPusher = new TalonFX(Constants.Climber.LEFT_PUSHER_PORT);
        rightClimber = new TalonFX(Constants.Climber.RIGHT_CLIMBER_PORT);
        // rightPusher = new TalonFX(Constants.Climber.RIGHT_PUSHER_PORT);

        // fixedClimbJoystick = new Joystick(Constants.Climber.FIXED_MANUAL_CLIMB_JOYSTICK_PORT);
        rotatingClimbJoystick = new Joystick(Constants.Climber.ROTATING_MANUAL_CLIMB_JOYSTICK_PORT);

        motors = new TalonFX[] {
            leftClimber, 
            rightClimber
        };

        lastLSState = new boolean[4];
        
        for(TalonFX motor: motors) {
            configureMotor(motor);
        }

        rightClimber.setInverted(true);
        //rightPusher.setInverted(true);
        leftClimber.setInverted(false);
        //leftPusher.setInverted(false);

        
        leftClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        rightClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        leftClimber.configForwardSoftLimitEnable(true);
        leftClimber.configForwardSoftLimitThreshold(Constants.Climber.FORWARD_SOFT_LIMIT);
        rightClimber.configForwardSoftLimitEnable(true);
        rightClimber.configForwardSoftLimitThreshold(Constants.Climber.FORWARD_SOFT_LIMIT);
        leftClimber.configReverseSoftLimitEnable(true);
        leftClimber.configReverseSoftLimitThreshold(Constants.Climber.REVERSE_SOFT_LIMIT);
        rightClimber.configReverseSoftLimitEnable(true);
        rightClimber.configReverseSoftLimitThreshold(Constants.Climber.REVERSE_SOFT_LIMIT);

        // motorLeft1.configForwardSoftLimitEnable(true);
        // motorLeft1.configForwardSoftLimitThreshold(145500);
        // motorLeft1.configReverseSoftLimitEnable(true);
        // motorLeft1.configReverseSoftLimitThreshold(3000);
        // motorLeft2.configForwardSoftLimitEnable(true);
        // motorLeft2.configForwardSoftLimitThreshold(170000);
        // motorRight1.configForwardSoftLimitEnable(true);
        // motorRight1.configForwardSoftLimitThreshold(145500);
        // motorRight1.configReverseSoftLimitEnable(true);
        // motorRight1.configReverseSoftLimitThreshold(3000);
        // motorRight2.configForwardSoftLimitEnable(true);
        // motorRight2.configForwardSoftLimitThreshold(170000);

       
        // motorLeft1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        // motorLeft2.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       
        // motorRight1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        // motorRight2.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        for(TalonFX motor: new TalonFX[]{leftClimber, rightClimber}) {
            motor.config_kF(0, Constants.Climber.KF_CLIMBER, Constants.TIMEOUT);
            motor.config_kP(0, Constants.Climber.KP_CLIMBER, Constants.TIMEOUT);
            motor.config_kI(0, Constants.Climber.KI_CLIMBER, Constants.TIMEOUT);
            motor.config_kD(0, Constants.Climber.KD_CLIMBER, Constants.TIMEOUT);
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

    // public void enableLowerSoftLimits(){
    //     motorLeft1.configReverseSoftLimitEnable(true);
    //     motorRight1.configReverseSoftLimitEnable(true);
    // }

    // public void disableLowerSoftLimits(){
    //     motorLeft1.configReverseSoftLimitEnable(false);
    //     motorRight1.configReverseSoftLimitEnable(false);
    // }

    public void setSpeed(ClimberMotor motor, double speed) {
        TalonFX talon = getMotor(motor);
        if(Math.abs(speed) <= 1) {
            talon.set(ControlMode.PercentOutput, speed);
        } else {
            talon.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setDiagnosticSpeed(ClimberMotor motor, double speed, ControlMode controlMode){
        TalonFX talon = getMotor(motor);
        if(controlMode.equals(ControlMode.PercentOutput)){
            if(Math.abs(speed) <= 1) {
                talon.set(controlMode, speed);
            } else {
                talon.set(ControlMode.PercentOutput, 0);
            }
        }
        if(controlMode.equals(ControlMode.Velocity)){
            if(Math.abs(speed) <= 1) {
                talon.set(controlMode, Units.percent2Velocity(speed));
            } else {
                talon.set(ControlMode.PercentOutput, 0);
            }
            
        }
        else{
            talon.set(controlMode, speed);
        }

    }

    public void zeroLeftClimber(){
        leftClimber.setSelectedSensorPosition(0);
    }

    public void zeroRightClimber(){
        rightClimber.setSelectedSensorPosition(0);
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

    public void setLimitSwitchEnable(boolean tRue){
        leftClimber.configForwardSoftLimitEnable(tRue);
        rightClimber.configForwardSoftLimitEnable(tRue);
        leftClimber.configReverseSoftLimitEnable(tRue);
        rightClimber.configReverseSoftLimitEnable(tRue);
    }

    // public void deployClimb(){
    //     TalonFX leftClimber = getMotor(ClimberMotor.LEFT_1);
    //     TalonFX rightClimber = getMotor(ClimberMotor.RIGHT_1);
    //     leftClimber.configMotionAcceleration(Constants.Climber.DEPLOY_ACCELERATION);
    //     leftClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_CRUISE_VELOCITY);

    //     rightClimber.configMotionAcceleration(Constants.Climber.DEPLOY_ACCELERATION);
    //     rightClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_CRUISE_VELOCITY);

    //     leftClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_LEFT_POS);
    //     rightClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_RIGHT_POS);
    // }

    // public void undeployClimb(){
    //     TalonFX leftClimber = getMotor(ClimberMotor.LEFT_1);
    //     TalonFX rightClimber = getMotor(ClimberMotor.RIGHT_1);
    //     leftClimber.configMotionAcceleration(Constants.Climber.RETRACT_ACCELERATION);
    //     leftClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_CRUISE_VELOCITY);

    //     rightClimber.configMotionAcceleration(Constants.Climber.RETRACT_ACCELERATION);
    //     rightClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_CRUISE_VELOCITY);

    //     leftClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_POS);
    //     rightClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_POS);
    // }

    // public void mantainPositionRetract(ClimberMotor motor) {
    //     TalonFX talon = getMotor(motor);

    //     talon.set(ControlMode.Position, Constants.Climber.RETRACT_POS);
    // }

    // public void mantainPositionRetract() {
    //     mantainPositionRetract(ClimberMotor.LEFT_1);
    //     mantainPositionRetract(ClimberMotor.RIGHT_1);
    // }
    
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
        double MAX_CURRENT = 20;

        double currentLeft1 = Math.abs(leftClimber.getStatorCurrent());
        // double currentLeft2 = Math.abs(leftPusher.getStatorCurrent());
        double currentRight1 = Math.abs(rightClimber.getStatorCurrent());
        // double currentRight2 = Math.abs(rightPusher.getStatorCurrent());

        return currentLeft1 > MAX_CURRENT
        // || currentLeft2 > MAX_CURRENT
        || currentRight1 > MAX_CURRENT;
        // || currentRight2 > MAX_CURRENT;
    }


    public void manualFixedClimb(){
        // getMotor(ClimberMotor.LEFT_1).set(ControlMode.PercentOutput, getFixedValue());
        // getMotor(ClimberMotor.RIGHT_1).set(ControlMode.PercentOutput, getFixedValue());
    }

    public double getClimberJoystickValue(){
        double rawX = rotatingClimbJoystick.getX();
        double value = rawX;
        return value;
    }

    public void manualRotatingClimb(){
        // getMotor(ClimberMotor.LEFT_1).set(ControlMode.PercentOutput, getRotatingValue());
        // getMotor(ClimberMotor.RIGHT_1).set(ControlMode.PercentOutput, getRotatingValue());
    }

    

    public void undeployClimber(){
        leftClimber = getMotor(ClimberMotor.LEFT_CLIMBER);
        rightClimber = getMotor(ClimberMotor.RIGHT_CLIMBER);

        leftClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_LEFT_CRUISE_VELOCITY);
        leftClimber.configMotionAcceleration(Constants.Climber.RETRACT_LEFT_ACCELERATION);

        rightClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_RIGHT_CRUISE_VELOCITY);
        rightClimber.configMotionAcceleration(Constants.Climber.RETRACT_RIGHT_ACCELERATION);

        leftClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_LEFT_POS);
        rightClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_RIGHT_POS);        
    }

    public void deployClimber(){
        leftClimber = getMotor(ClimberMotor.LEFT_CLIMBER);
        rightClimber = getMotor(ClimberMotor.RIGHT_CLIMBER);

        leftClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_LEFT_CRUISE_VELOCITY);
        leftClimber.configMotionAcceleration(Constants.Climber.DEPLOY_LEFT_ACCELERATION);

        rightClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_RIGHT_CRUISE_VELOCITY);
        rightClimber.configMotionAcceleration(Constants.Climber.DEPLOY_RIGHT_ACCELERATION);

        leftClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_LEFT_POS);
        rightClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_RIGHT_POS);
    }

    //After the robot has been pushed up, the climbers release, allowing it to fall - DIFFERENT FROM UNDEPLOY
    public void releaseClimber(){
        leftClimber = getMotor(ClimberMotor.LEFT_CLIMBER);
        rightClimber = getMotor(ClimberMotor.RIGHT_CLIMBER);

        leftClimber.configMotionCruiseVelocity(Constants.Climber.RELEASE_LEFT_CRUISE_VELOCITY);
        leftClimber.configMotionAcceleration(Constants.Climber.RELEASE_LEFT_ACCELERATION);

        rightClimber.configMotionCruiseVelocity(Constants.Climber.RELEASE_RIGHT_CRUISE_VELOCITY);
        rightClimber.configMotionAcceleration(Constants.Climber.RELEASE_RIGHT_ACCELERATION);

        leftClimber.set(ControlMode.MotionMagic, Constants.Climber.RELEASE_LEFT_POS);
        rightClimber.set(ControlMode.MotionMagic, Constants.Climber.RELEASE_RIGHT_POS);
    }

    public void zeroSensors(){
        for(TalonFX motor : motors){
            motor.setSelectedSensorPosition(0);
        }
    }


    public void setF(double kP){
        getMotor(ClimberMotor.LEFT_CLIMBER).config_kF(0, kP);
        getMotor(ClimberMotor.RIGHT_CLIMBER).config_kF(0, kP);
    }

    public void setP(double kI){
        getMotor(ClimberMotor.LEFT_CLIMBER).config_kP(0, kI);
        getMotor(ClimberMotor.RIGHT_CLIMBER).config_kP(0, kI);
    }

    public void setI(double kD){
        getMotor(ClimberMotor.LEFT_CLIMBER).config_kI(0, kD);
        getMotor(ClimberMotor.RIGHT_CLIMBER).config_kI(0, kD);
    }

    public void setD(double kF){
        getMotor(ClimberMotor.LEFT_CLIMBER).config_kD(0, kF);
        getMotor(ClimberMotor.RIGHT_CLIMBER).config_kD(0, kF);
    }

    public void setCruiseVelocity(ClimberMotor motor, double velocity){
        getMotor(motor).configMotionCruiseVelocity(velocity);
    }

    public void setAcceleration(ClimberMotor motor, double acceleration){
        getMotor(motor).configMotionAcceleration(acceleration);
    }
}
