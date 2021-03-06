package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Units;

public class PusherSubsystem extends SubsystemBase{
    private TalonFX leftPusher;
    private TalonFX rightPusher;
    private Joystick fixedClimbJoystick;

    private TalonFX[] motors;

    public enum PusherMotor {
        LEFT_PUSHER,
        RIGHT_PUSHER
    }

    public PusherSubsystem(){
        leftPusher = new TalonFX(Constants.Climber.LEFT_PUSHER_PORT);
        rightPusher = new TalonFX(Constants.Climber.RIGHT_PUSHER_PORT);

        fixedClimbJoystick = new Joystick(Constants.Climber.FIXED_MANUAL_CLIMB_JOYSTICK_PORT);

        motors = new TalonFX[] {leftPusher,
                                rightPusher};

        rightPusher.setInverted(true);
        leftPusher.setInverted(false);

        for(TalonFX motor: new TalonFX[]{leftPusher, rightPusher}) {
            configureMotor(motor);
            motor.config_kF(0, Constants.Climber.KF_PUSHER, Constants.TIMEOUT);
            motor.config_kP(0, Constants.Climber.KP_PUSHER, Constants.TIMEOUT);
            motor.config_kI(0, Constants.Climber.KI_PUSHER, Constants.TIMEOUT);
            motor.config_kD(0, Constants.Climber.KD_PUSHER, Constants.TIMEOUT);
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

    private static PusherSubsystem singleton;
    public static PusherSubsystem getInstance(){
        if(singleton == null) singleton = new PusherSubsystem();
        return singleton;
    }

    private TalonFX getMotor(PusherMotor motor) {
        return motors[motor.ordinal()];
    }

    public boolean isCurrentBad() {
        double MAX_CURRENT = 20;

        double currentLeft2 = Math.abs(leftPusher.getStatorCurrent());
        double currentRight2 = Math.abs(rightPusher.getStatorCurrent());

        return currentLeft2 > MAX_CURRENT
        || currentRight2 > MAX_CURRENT;
    }

    public void setSpeed(PusherMotor motor, double speed) {
        TalonFX talon = getMotor(motor);
        if(Math.abs(speed) <= 1) {
            talon.set(ControlMode.PercentOutput, speed);
        } else {
            talon.set(ControlMode.PercentOutput, 0);
        }
    }

    public void seLefttMotionMagic(double pos){
        leftPusher = getMotor(PusherMotor.LEFT_PUSHER);

        leftPusher.configMotionCruiseVelocity(Constants.Climber.RELEASE_LEFT_PUSHER_CRUISE_VELOCITY_SLOW);
        leftPusher.configMotionAcceleration(Constants.Climber.RELEASE_LEFT_PUSHER_ACCELERATION_SLOW);

        leftPusher.set(ControlMode.MotionMagic, pos);
    }

    public void setRighttMotionMagic(double pos){
        rightPusher = getMotor(PusherMotor.RIGHT_PUSHER);
        
        rightPusher.configMotionCruiseVelocity(Constants.Climber.RELEASE_RIGHT_PUSHER_CRUISE_VELOCITY_SLOW);
        rightPusher.configMotionAcceleration(Constants.Climber.RELEASE_RIGHT_PUSHER_ACCELERATION_SLOW);

        rightPusher.set(ControlMode.MotionMagic, pos);
    }

    public void setDiagnosticSpeed(PusherMotor motor, double speed, ControlMode controlMode){
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


    public double getFixedValue(){
        double rawY = fixedClimbJoystick.getY();
        double value = rawY*0.25;
        return value;
    }

    public double getPosition(PusherMotor motor) {
        return getMotor(motor).getSelectedSensorPosition();
    }

    public double getStatorCurrent(PusherMotor motor) {
        return getMotor(motor).getStatorCurrent();
    }

    public double getSupplyCurrent(PusherMotor motor) {
        return getMotor(motor).getSupplyCurrent();
    }

    public void stop() {
        for(TalonFX motor: motors) {
            motor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void deployPusher(){
        leftPusher = getMotor(PusherMotor.LEFT_PUSHER);
        rightPusher = getMotor(PusherMotor.RIGHT_PUSHER);

        leftPusher.configMotionCruiseVelocity(Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        leftPusher.configMotionAcceleration(Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        rightPusher.configMotionCruiseVelocity(Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        rightPusher.configMotionAcceleration(Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        leftPusher.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_LEFT_PUSHER_POS);
        rightPusher.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_RIGHT_PUSHER_POS);
    }

    public void push(){
        leftPusher = getMotor(PusherMotor.LEFT_PUSHER);
        rightPusher = getMotor(PusherMotor.RIGHT_PUSHER);

        leftPusher.configMotionCruiseVelocity(Constants.Climber.PUSH_LEFT_CRUISE_VELOCITY);
        leftPusher.configMotionAcceleration(Constants.Climber.PUSH_LEFT_ACCELERATION);

        rightPusher.configMotionCruiseVelocity(Constants.Climber.PUSH_RIGHT_CRUISE_VELOCITY);
        rightPusher.configMotionAcceleration(Constants.Climber.PUSH_RIGHT_ACCELERATION);

        leftPusher.set(ControlMode.MotionMagic, Constants.Climber.PUSH_LEFT_POS);
        rightPusher.set(ControlMode.MotionMagic, Constants.Climber.PUSH_RIGHT_POS);
    }

    public void releasePusherSlow(){
        leftPusher = getMotor(PusherMotor.LEFT_PUSHER);
        rightPusher = getMotor(PusherMotor.RIGHT_PUSHER);

        leftPusher.configMotionCruiseVelocity(Constants.Climber.RELEASE_LEFT_PUSHER_CRUISE_VELOCITY_SLOW);
        leftPusher.configMotionAcceleration(Constants.Climber.RELEASE_LEFT_PUSHER_ACCELERATION_SLOW);
        
        rightPusher.configMotionCruiseVelocity(Constants.Climber.RELEASE_RIGHT_PUSHER_CRUISE_VELOCITY_SLOW);
        rightPusher.configMotionAcceleration(Constants.Climber.RELEASE_RIGHT_PUSHER_ACCELERATION_SLOW);

        leftPusher.set(ControlMode.MotionMagic, Constants.Climber.RELEASE_LEFT_PUSHER_SLOW_POS);
        rightPusher.set(ControlMode.MotionMagic, Constants.Climber.RELEASE_RIGHT_PUSHER_SLOW_POS);
    }

    public void releasePusherFast(){
        leftPusher = getMotor(PusherMotor.LEFT_PUSHER);
        rightPusher = getMotor(PusherMotor.RIGHT_PUSHER);

        leftPusher.configMotionCruiseVelocity(Constants.Climber.RELEASE_LEFT_PUSHER_CRUISE_VELOCITY_FAST);
        leftPusher.configMotionAcceleration(Constants.Climber.RELEASE_LEFT_PUSHER_ACCELERATION_FAST);
        
        rightPusher.configMotionCruiseVelocity(Constants.Climber.RELEASE_RIGHT_PUSHER_CRUISE_VELOCITY_FAST);
        rightPusher.configMotionAcceleration(Constants.Climber.RELEASE_RIGHT_PUSHER_ACCELERATION_FAST);

        leftPusher.set(ControlMode.MotionMagic, Constants.Climber.RELEASE_LEFT_PUSHER_FAST_POS);
        rightPusher.set(ControlMode.MotionMagic, Constants.Climber.RELEASE_RIGHT_PUSHER_FAST_POS);
    }

    public void resetPusher(){
        leftPusher = getMotor(PusherMotor.LEFT_PUSHER);
        rightPusher = getMotor(PusherMotor.RIGHT_PUSHER);

        leftPusher.configMotionCruiseVelocity(Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        leftPusher.configMotionAcceleration(Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        rightPusher.configMotionCruiseVelocity(Constants.Climber.DEPLOY_PUSH_CRUISE_VELOCITY);
        rightPusher.configMotionAcceleration(Constants.Climber.DEPLOY_PUSH_ACCELERATION);

        leftPusher.set(ControlMode.MotionMagic, 0);
        rightPusher.set(ControlMode.MotionMagic, 0);
    }

    public void zeroLeftPusher(){
        leftPusher.setSelectedSensorPosition(0);
    }

    public void zeroRightPusher(){
        rightPusher.setSelectedSensorPosition(0);
    }

    public void zeroPushers(){
        zeroLeftPusher();
        zeroRightPusher();
    }

    public void setF(double kP){
        getMotor(PusherMotor.LEFT_PUSHER).config_kF(0, kP);
        getMotor(PusherMotor.RIGHT_PUSHER).config_kF(0, kP);
    }

    public void setP(double kI){
        getMotor(PusherMotor.LEFT_PUSHER).config_kP(0, kI);
        getMotor(PusherMotor.RIGHT_PUSHER).config_kP(0, kI);
    }

    public void setI(double kD){
        getMotor(PusherMotor.LEFT_PUSHER).config_kI(0, kD);
        getMotor(PusherMotor.RIGHT_PUSHER).config_kI(0, kD);
    }

    public void setD(double kF){
        getMotor(PusherMotor.LEFT_PUSHER).config_kD(0, kF);
        getMotor(PusherMotor.RIGHT_PUSHER).config_kD(0, kF);
    }

    public void setCruiseVelocity(PusherMotor motor, double velocity){
        getMotor(motor).configMotionCruiseVelocity(velocity);
    }

    public void setAcceleration(PusherMotor motor, double acceleration){
        getMotor(motor).configMotionAcceleration(acceleration);
    }
}
