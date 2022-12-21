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

    // private Joystick fixedClimbJoystick;
    private Joystick rotatingClimbJoystick;

    private TalonFX getMotor(ClimberMotor motor) {
        return motors[motor.ordinal()];
    }

    public boolean getLeftLimitSwitch() {
         return leftClimber.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public boolean getRightLimitSwitch(){
        return rightClimber.getSensorCollection().isRevLimitSwitchClosed() == 1;
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
        
        for(TalonFX motor: motors) {
            configureMotor(motor);
        }

        rightClimber.setInverted(true);
        //rightPusher.setInverted(true);
        leftClimber.setInverted(false);
        //leftPusher.setInverted(false);

        
        //leftClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        //rightClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

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

    public void setLeftSpeed(double speed) {
        if(Math.abs(speed) <= 1) {
            leftClimber.set(ControlMode.PercentOutput, speed);
        } else {
            leftClimber.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setRightSpeed(double speed){
        if(Math.abs(speed) <= 1) {
            rightClimber.set(ControlMode.PercentOutput, speed);
        } else {
            rightClimber.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setSpeed(double speed){
        setLeftSpeed(speed);
        setRightSpeed(speed);
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
    
    public double getLeftPosition() {
        return leftClimber.getSelectedSensorPosition();
    }

    public double getRightPosition(){
        return rightClimber.getSelectedSensorPosition();
    }

    public double getLeftStatorCurrent() {
        return leftClimber.getStatorCurrent();
    }

    public double getRightStatorCurrent() {
        return rightClimber.getStatorCurrent();
    }

    public double getLeftSupplyCurrent() {
        return leftClimber.getSupplyCurrent();
    }

    public double getRightSupplyCurrent() {
        return rightClimber.getSupplyCurrent();
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

    public double getClimberJoystickValue(){
        double rawX = rotatingClimbJoystick.getX();
        double value = rawX;
        return value;
    }

    public void undeployClimber(){
        leftClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_LEFT_CRUISE_VELOCITY);
        leftClimber.configMotionAcceleration(Constants.Climber.RETRACT_LEFT_ACCELERATION);

        rightClimber.configMotionCruiseVelocity(Constants.Climber.RETRACT_RIGHT_CRUISE_VELOCITY);
        rightClimber.configMotionAcceleration(Constants.Climber.RETRACT_RIGHT_ACCELERATION);

        leftClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_LEFT_POS);
        rightClimber.set(ControlMode.MotionMagic, Constants.Climber.RETRACT_RIGHT_POS);        
    }

    public void deployClimber(){
        leftClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_LEFT_CRUISE_VELOCITY);
        leftClimber.configMotionAcceleration(Constants.Climber.DEPLOY_LEFT_ACCELERATION);

        rightClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_RIGHT_CRUISE_VELOCITY);
        rightClimber.configMotionAcceleration(Constants.Climber.DEPLOY_RIGHT_ACCELERATION);

        leftClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_LEFT_POS);
        rightClimber.set(ControlMode.MotionMagic, Constants.Climber.DEPLOY_RIGHT_POS);
    }

    //After the robot has been pushed up, the climbers release, allowing it to fall - DIFFERENT FROM UNDEPLOY
    public void releaseClimber(){
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
        leftClimber.config_kF(0, kP);
        rightClimber.config_kF(0, kP);
    }

    public void setP(double kI){
        leftClimber.config_kP(0, kI);
        rightClimber.config_kP(0, kI);
    }

    public void setI(double kD){
        leftClimber.config_kI(0, kD);
        rightClimber.config_kI(0, kD);
    }

    public void setD(double kF){
        leftClimber.config_kD(0, kF);
        rightClimber.config_kD(0, kF);
    }

    public void setCruiseVelocity(ClimberMotor motor, double velocity){
        getMotor(motor).configMotionCruiseVelocity(velocity);
    }

    public void setAcceleration(ClimberMotor motor, double acceleration){
        getMotor(motor).configMotionAcceleration(acceleration);
    }

    public void setPosition(int leftPos, int rightPos){
        leftClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_LEFT_CRUISE_VELOCITY);
        leftClimber.configMotionAcceleration(Constants.Climber.DEPLOY_LEFT_ACCELERATION);

        rightClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_RIGHT_CRUISE_VELOCITY);
        rightClimber.configMotionAcceleration(Constants.Climber.DEPLOY_RIGHT_ACCELERATION);

        leftClimber.set(ControlMode.MotionMagic, leftPos);
        rightClimber.set(ControlMode.MotionMagic, rightPos);
    }

    public void setPosition(int pos){
        setPosition(pos, pos);
    }

    public void leftSetPos(int pos){
        leftClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_LEFT_CRUISE_VELOCITY);
        leftClimber.configMotionAcceleration(Constants.Climber.DEPLOY_LEFT_ACCELERATION);

        leftClimber.set(ControlMode.MotionMagic, pos);
    }

    public void rightSetPos(int pos){
        rightClimber.configMotionCruiseVelocity(Constants.Climber.DEPLOY_RIGHT_CRUISE_VELOCITY);
        rightClimber.configMotionAcceleration(Constants.Climber.DEPLOY_RIGHT_ACCELERATION);

        rightClimber.set(ControlMode.MotionMagic, pos);
    }
}
