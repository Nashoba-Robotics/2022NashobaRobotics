package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.OdometryCarpetCompensator;
import frc.robot.lib.Units;

// Subsystem for driving the robot
public class DriveSubsystem extends SubsystemBase {

    public enum DriveMode {
        VELOCITY,
        PERCENT
    }

    private static DriveSubsystem instance;

    //x-speed, y-speed, rate of rotation
    private Rotation2d gyroAngle = Rotation2d.fromDegrees(0);
    private OdometryCarpetCompensator odometry;
    private Pose2d pose;
    private double angOfResistance;

    public static final int VOLTAGE_COMPENSATION_LEVEL = 12;
    //public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // find
    public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; // find this

    private DriveMode driveMode = DriveMode.VELOCITY;

    private boolean brakeMode = false;

    private TalonFX leftMotor, leftMotor2, leftMotor3;
    private TalonFX rightMotor, rightMotor2, rightMotor3;

    private double lastRightNU;
    private double lastLeftNU;

    private boolean odometryResetFinished;

    @Override
    public void periodic() {
        if(RobotState.isAutonomous() && RobotState.isEnabled()){
            if(odometryResetFinished && odometry != null){
                gyroAngle = Rotation2d.fromDegrees(GyroSubsystem.getInstance().getAbsoluteAngle());
                double currLeft = getPositionLeft();
                double currRight = getPositionRight();
                double deltaLeft = currLeft - lastLeftNU;
                double deltaRight = currRight - lastRightNU;
                // SmartDashboard.putNumber("l delta", deltaLeft);
                // SmartDashboard.putNumber("r delta", deltaRight);
                pose = odometry.updatePose2d(
                    gyroAngle,
                    deltaLeft,
                    deltaRight);

                lastLeftNU = currLeft;
                lastRightNU = currRight;

                SmartDashboard.putNumber("angle odometry", odometry.getAngle());
                SmartDashboard.putNumber("angle gyro", GyroSubsystem.getInstance().getAbsoluteAngle());
                SmartDashboard.putNumber("l odo", odometry.getLeftNU());
                SmartDashboard.putNumber("r odo", odometry.getRightNU());

                SmartDashboard.putNumber("Odometry X", getPose().getX());
                SmartDashboard.putNumber("Odometry Y", getPose().getY());
            }

            SmartDashboard.putNumber("l NU", getPositionLeft());
            SmartDashboard.putNumber("r NU", getPositionRight());
        } else {
            odometryResetFinished = false;
        }
    }
    
    public DriveSubsystem() {
        odometryResetFinished = false;

        leftMotor = new TalonFX(Constants.LEFT_MOTOR_PORTS[0], "Drive");
        leftMotor2 = new TalonFX(Constants.LEFT_MOTOR_PORTS[1], "Drive");
        leftMotor3 = new TalonFX(Constants.LEFT_MOTOR_PORTS[2], "Drive");
        rightMotor = new TalonFX(Constants.RIGHT_MOTOR_PORTS[0], "Drive");
        rightMotor2 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[1], "Drive");
        rightMotor3 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[2], "Drive");

        // leftMotor = new TalonFX(Constants.LEFT_MOTOR_PORTS[0]);
        // leftMotor2 = new TalonFX(Constants.LEFT_MOTOR_PORTS[1]);
        // leftMotor3 = new TalonFX(Constants.LEFT_MOTOR_PORTS[2]);
        // rightMotor = new TalonFX(Constants.RIGHT_MOTOR_PORTS[0]);
        // rightMotor2 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[1]);
        // rightMotor3 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[2]);

        leftMotor2.follow(leftMotor);
        leftMotor3.follow(leftMotor);
        rightMotor2.follow(rightMotor);
        rightMotor3.follow(rightMotor);
        brakeMode = false;

        configureMotor(rightMotor);
        configureMotor(leftMotor);

        leftMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        rightMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);

        rightMotor.config_kF(Constants.SLOT_IDX, Constants.DriveTrain.KF_RIGHT, Constants.TIMEOUT);
		rightMotor.config_kP(Constants.SLOT_IDX, Constants.DriveTrain.P_RIGHT, Constants.TIMEOUT);
		rightMotor.config_kI(Constants.SLOT_IDX, Constants.DriveTrain.I_RIGHT, Constants.TIMEOUT);
        rightMotor.config_kD(Constants.SLOT_IDX, Constants.DriveTrain.D_RIGHT, Constants.TIMEOUT);

        leftMotor.config_kF(Constants.SLOT_IDX, Constants.DriveTrain.KF_LEFT, Constants.TIMEOUT);
		leftMotor.config_kP(Constants.SLOT_IDX, Constants.DriveTrain.P_LEFT, Constants.TIMEOUT);
		leftMotor.config_kI(Constants.SLOT_IDX, Constants.DriveTrain.I_LEFT, Constants.TIMEOUT);
        leftMotor.config_kD(Constants.SLOT_IDX, Constants.DriveTrain.D_LEFT, Constants.TIMEOUT);
        
        leftMotor.setInverted(InvertType.InvertMotorOutput);
        leftMotor2.setInverted(InvertType.FollowMaster);
        leftMotor3.setInverted(InvertType.FollowMaster);
        rightMotor.setInverted(InvertType.None);
        rightMotor2.setInverted(InvertType.FollowMaster);
        rightMotor3.setInverted(InvertType.FollowMaster);

        // Set the name of the subsystem in smart dashboard
        SendableRegistry.setName(this, "Drive");
    }

    public static DriveSubsystem getInstance(){
        if(instance == null){
            instance = new DriveSubsystem();
        }
        return instance;
    }

    public void createOdometry(double angOfResistance, double startAngle){
        this.angOfResistance = angOfResistance;
        odometry = new OdometryCarpetCompensator(angOfResistance, Rotation2d.fromDegrees(startAngle*360/Constants.TAU));
    }

    // public void resetOdometryTrue(){
    //     odometryResetFinished = false;
    // }

    public void setMetersPerSecond(double left, double right){
        leftMotor.set(ControlMode.Velocity, Units.meters2NUSpeed(left));
        rightMotor.set(ControlMode.Velocity, Units.meters2NUSpeed(right));
    }

    public void setVolts(double left, double right){
        leftMotor.set(ControlMode.PercentOutput, left / 12);
        rightMotor.set(ControlMode.PercentOutput, right / 12);
    }

    public void resetOdometry(Pose2d pose, double angOfResistance){
        odometryResetFinished = false;
        this.angOfResistance = Units.getAbsAngle(angOfResistance);
        if(odometry == null){
            odometry = new OdometryCarpetCompensator(angOfResistance, pose.getRotation());
            odometry.resetPos(pose, pose.getRotation());
        } else {
            odometry.resetPos(pose, pose.getRotation(), angOfResistance);
        }
        lastLeftNU = 0;
        lastRightNU = 0;
        leftMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        rightMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        GyroSubsystem.getInstance().setAngle(pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Reset Angle", odometry.getPoseMeters().getRotation().getRadians());
        odometryResetFinished = true;
    }


    public void resetOdometry(Pose2d pose){
        odometryResetFinished = false;
        if(odometry == null){
            odometry = new OdometryCarpetCompensator(angOfResistance, pose.getRotation());
            odometry.resetPos(pose, pose.getRotation());
        } else {
            odometry.resetPos(pose, pose.getRotation(), angOfResistance);
        }
        lastLeftNU = 0;
        lastRightNU = 0;
        leftMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        rightMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        GyroSubsystem.getInstance().setAngle(pose.getRotation().getRadians());
        SmartDashboard.putNumber("Reset Angle", odometry.getPoseMeters().getRotation().getRadians());
        odometryResetFinished = true;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(Units.NU2Meters(getLeftMotorVelocity() * 10), Units.NU2Meters(getRightMotorVelocity() * 10));
    }

    public double getAngle(){
        return getPose().getRotation().getRadians();
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public double getTranslationX(){
        return getPose().getTranslation().getX();
    }

    public double getTranslationY(){
        return getPose().getTranslation().getY();
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

        motor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        motor.clearStickyFaults();
    }

    public void changeBrakeMode(){
        brakeMode = !brakeMode;
    }

    public boolean getBrakeMode(){
        return brakeMode;
    }

    public void setBrakeMode(boolean brakeMode){
        this.brakeMode = brakeMode;
    }

    //Changes the netural mode of both sets of drive motors (Defaults as Coast);
    public void changeNeutralMode(NeutralMode n){
        leftMotor.setNeutralMode(n);
        rightMotor.setNeutralMode(n);
    }

    public void setProportional(double p){
        rightMotor.config_kP(Constants.SLOT_IDX, p, Constants.TIMEOUT);
        leftMotor.config_kP(Constants.SLOT_IDX, p, Constants.TIMEOUT);
    }

    public void setIntegral(double i){
        rightMotor.config_kI(Constants.SLOT_IDX, i, Constants.TIMEOUT);
        leftMotor.config_kI(Constants.SLOT_IDX, i, Constants.TIMEOUT);
    }

    public void setDerivative(double d){
        rightMotor.config_kD(Constants.SLOT_IDX, d, Constants.TIMEOUT);
        leftMotor.config_kD(Constants.SLOT_IDX, d, Constants.TIMEOUT);
    }

    //takes input, speed, in form of percent (-1 through 1). Sets the speed of the right motor
    public void setRightMotorSpeed(double speed){
        TalonFXControlMode controlMode = TalonFXControlMode.Velocity;

        if(driveMode == DriveMode.VELOCITY){
            controlMode = TalonFXControlMode.Velocity;
            speed = Units.percent2Velocity(speed);
        } else if(driveMode == DriveMode.PERCENT){
            controlMode = TalonFXControlMode.PercentOutput;
        }
        
        double aff = Constants.DriveTrain.AFF_RIGHT * Math.signum(speed);
        
        if(!brakeMode){
            rightMotor.set(controlMode, speed, DemandType.ArbitraryFeedForward, aff);
        }else {
            rightMotor.set(ControlMode.PercentOutput, 0);
        }
        
    }

    //takes input, speed, in form of percent (-1 through 1). Sets the speed of the left motor
    public void setLeftMotorSpeed(double speed){
        TalonFXControlMode controlMode = TalonFXControlMode.Velocity;

        if(driveMode == DriveMode.VELOCITY){
            controlMode = TalonFXControlMode.Velocity;
            speed = Units.percent2Velocity(speed);
        } else if(driveMode == DriveMode.PERCENT){
            controlMode = TalonFXControlMode.PercentOutput;
        }

        double aff = Constants.DriveTrain.AFF_LEFT * Math.signum(speed);

        leftMotor.set(controlMode, speed, DemandType.ArbitraryFeedForward, aff);

        if(!brakeMode){
            leftMotor.set(controlMode, speed, DemandType.ArbitraryFeedForward, aff);
        }else {
            leftMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    // Set the speed of both the left and right side
    public void setSpeed(double speed) {
        setSpeed(speed, speed);
    }

    // Set the left and right sides separately
    public void setSpeed(double left, double right) {
        setLeftMotorSpeed(left);
        setRightMotorSpeed(right);
    }

    public void setRawMotorVelocity(double left, double right){
        leftMotor.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, Constants.DriveTrain.AFF_LEFT);
        rightMotor.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, Constants.DriveTrain.AFF_RIGHT);
    }

    public void setRawPercent(double left, double right){
        leftMotor.set(ControlMode.PercentOutput, left);
        rightMotor.set(ControlMode.PercentOutput, right);
    }

    public double getLeftMotorVelocity(){
        return leftMotor.getSelectedSensorVelocity(Constants.PID_IDX);
    }

    public double getRightMotorVelocity(){
        return rightMotor.getSelectedSensorVelocity(Constants.PID_IDX);
    }
    
    public double getLeftMotorCurrent(){
        return leftMotor.getStatorCurrent();
    }

    public double getRightMotorCurrent(){
        return rightMotor.getStatorCurrent();
    }

    public double getLeftMotorError() {
        return leftMotor.getClosedLoopError();
    }

    public double getRightMotorError() {
        return rightMotor.getClosedLoopError();
    }

    // Input: DriveMode enum. Sets drive mode the robot will be in (VELOCITY/PERCENT)
    public void setDriveMode(DriveMode mode){
        driveMode = mode;
    }

    public DriveMode getDriveMode(){
        return driveMode;
    }

    public double getPositionLeft(){
        return leftMotor.getSelectedSensorPosition(Constants.PID_IDX);
    }

    public double getPositionRight(){
        return rightMotor.getSelectedSensorPosition(Constants.PID_IDX);
    }

    public double getDistanceLeft(){
        return Units.NU2Meters(getPositionLeft());
    }

    public double getDistanceRight(){
        return Units.NU2Meters(getPositionRight());
    }

    public double[] turnToAngle(double angle){  //in degrees
        double[] targetPoses = new double[2];
        leftMotor.configMotionCruiseVelocity(18_000);
        leftMotor.configMotionAcceleration(30_000);

        double targetPos = -332 * angle + leftMotor.getSelectedSensorPosition();
        leftMotor.set(ControlMode.MotionMagic, targetPos);
        targetPoses[0] = targetPos;

        rightMotor.configMotionCruiseVelocity(18_000);
        rightMotor.configMotionAcceleration(30_000);

        targetPos = 277 * angle + rightMotor.getSelectedSensorPosition();
        rightMotor.set(ControlMode.MotionMagic, targetPos);
        targetPoses[1] = targetPos;

        return targetPoses;
    }

    public double getLeftTargetAimPos(double angle){
        return -332 * angle + leftMotor.getSelectedSensorPosition();
    }

    public double getRightTargetAimPos(double angle){
        return 277 * angle + rightMotor.getSelectedSensorPosition();
    }
}
