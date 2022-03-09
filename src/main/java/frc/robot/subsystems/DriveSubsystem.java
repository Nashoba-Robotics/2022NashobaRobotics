package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Units;

// Subsystem for driving the robot
public class DriveSubsystem extends SubsystemBase {

    public enum DriveMode {
        VELOCITY,
        PERCENT
    }

    private static DriveSubsystem instance;

    //x-speed, y-speed, rate of rotation
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.WHEEL_GAP);
    private Rotation2d gyroAngle = Rotation2d.fromDegrees(-GyroSubsystem.getInstance().getAbsoluteAngle());
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyroAngle);
    private Pose2d pose;

    public static final int VOLTAGE_COMPENSATION_LEVEL = 12;
    //public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // find
    public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; // find this

    private DriveMode driveMode = DriveMode.VELOCITY;

    private boolean brakeMode = false;

    private TalonFX leftMotor, leftMotor2, leftMotor3;
    private TalonFX rightMotor, rightMotor2, rightMotor3;

    // @Override
    @Override
    public void periodic() {
        gyroAngle = Rotation2d.fromDegrees(-GyroSubsystem.getInstance().getAbsoluteAngle());
        pose = odometry.update(gyroAngle, NU2Meters(leftMotor.getSelectedSensorPosition(Constants.PID_IDX)), NU2Meters(rightMotor.getSelectedSensorPosition()));
    }

    public void setVoltage(double leftVolts, double rightVolts){
        leftVolts /= 13; //RobotController.getBatteryVoltage();
        rightVolts /= 13; //RobotController.getBatteryVoltage();
        // if(Math.abs(leftVolts) > Constants.DriveTrain.MAX_AUTO_TURN || Math.abs(rightVolts) > Constants.DriveTrain.MAX_AUTO_TURN) {
        //     double factor = Math.max(Math.abs(leftVolts), Math.abs(rightVolts));
        //     leftVolts /= factor;
        //     rightVolts /= factor;
        // }
        leftMotor.set(ControlMode.PercentOutput, leftVolts);
        rightMotor.set(ControlMode.PercentOutput, rightVolts);
    }
    //I see everything
    public void resetOdometry(Pose2d pose){
        leftMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        rightMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        odometry.resetPosition(pose, Rotation2d.fromDegrees(GyroSubsystem.getInstance().getAbsoluteAngle()));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(NU2Meters(getLeftMotorVelocity() * 10), NU2Meters(getRightMotorVelocity() * 10));
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

    private double NU2Meters(double nu){
        double rate = Constants.TAU * (Constants.WHEEL_RADIUS / Constants.DRIVE_GEAR_RATIO) / Constants.FALCON_NU;
        return nu * rate;
    }

    private Rotation2d toRotation2d(double angle){
        return Rotation2d.fromDegrees(angle);
    }

    public DifferentialDriveKinematics getKinematics(){
        return kinematics;
    }
    
    public DriveSubsystem() {
        leftMotor = new TalonFX(Constants.LEFT_MOTOR_PORTS[0]);
        leftMotor2 = new TalonFX(Constants.LEFT_MOTOR_PORTS[1]);
        leftMotor3 = new TalonFX(Constants.LEFT_MOTOR_PORTS[2]);
        rightMotor = new TalonFX(Constants.RIGHT_MOTOR_PORTS[0]);
        rightMotor2 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[1]);
        rightMotor3 = new TalonFX(Constants.RIGHT_MOTOR_PORTS[2]);

        leftMotor2.follow(leftMotor);
        leftMotor3.follow(leftMotor);
        rightMotor2.follow(rightMotor);
        rightMotor3.follow(rightMotor);
        brakeMode = false;

        leftMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);
        rightMotor.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.TIMEOUT);

        configureMotor(rightMotor);
        configureMotor(leftMotor);
        
        rightMotor.setInverted(InvertType.InvertMotorOutput);
        rightMotor2.setInverted(InvertType.FollowMaster);
        rightMotor3.setInverted(InvertType.FollowMaster);
        leftMotor.setInverted(InvertType.None);
        leftMotor2.setInverted(InvertType.FollowMaster);
        leftMotor3.setInverted(InvertType.FollowMaster);

        // Set the name of the subsystem in smart dashboard
        SendableRegistry.setName(this, "Drive");
    }

    public static DriveSubsystem getInstance(){
        if(instance == null){
            instance = new DriveSubsystem();
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

    public void changeBrakeMode(){
        brakeMode = !brakeMode;
    }
    //I also know where you live
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
        
        double aff = Constants.AFF * Math.signum(speed);
        
        if(!brakeMode){
            rightMotor.set(controlMode, speed, DemandType.ArbitraryFeedForward, aff);
        }else {
            rightMotor.set(ControlMode.PercentOutput, 0);
        }
        
    }
    //Hee hee
    //takes input, speed, in form of percent (-1 through 1). Sets the speed of the left motor
    public void setLeftMotorSpeed(double speed){
        TalonFXControlMode controlMode = TalonFXControlMode.Velocity;

        if(driveMode == DriveMode.VELOCITY){
            controlMode = TalonFXControlMode.Velocity;
            speed = Units.percent2Velocity(speed);
        } else if(driveMode == DriveMode.PERCENT){
            controlMode = TalonFXControlMode.PercentOutput;
        }

        double aff = Constants.AFF * Math.signum(speed);

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
        //left = Units.percent2Velocity(left);
        //right = Units.percent2Velocity(right);
        leftMotor.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, Constants.AFF);
        rightMotor.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, Constants.AFF);
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
        return -rightMotor.getSelectedSensorPosition(Constants.PID_IDX);
    }

    public double getDistanceLeft(){
        return NU2Meters(getPositionLeft());
    }

    public double getDistanceRight(){
        return NU2Meters(getPositionRight());
    }

    public Command getAutonomousCommand(String trajectoryJSON){

        //uncomment if setting points manually
        // DifferentialDriveVoltageConstraint autoVoltageConstraint = 
        // new DifferentialDriveVoltageConstraint(
        //   new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
        //   DriveSubsystem.getInstance().getKinematics(),
        //   7);
    
        //uncomment if setting points manually
        // TrajectoryConfig config =
        // new TrajectoryConfig(
        //   Constants.DriveTrain.MAX_VELOCITY,
        //   Constants.DriveTrain.MAX_ACCELERATION)
        //   .setKinematics(DriveSubsystem.getInstance().getKinematics())
        //   .addConstraint(autoVoltageConstraint);
    
        Trajectory trajectory = new Trajectory();
    
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
       } catch (IOException e) {
          DriverStation.reportError("Unable to open trajectory: \n \n \n \n", e.getStackTrace());
       }
    
        //uncomment if adding points manually
        // Trajectory trajectory =
        // TrajectoryGenerator.generateTrajectory(
        //   new Pose2d(0, 0, new Rotation2d(0)), //starting position
        //   List.of(new Translation2d(1, 1), new Translation2d(2, -1)), //nodes for robot to travel to
        //   new Pose2d(3, 0, new Rotation2d(0)), //finishing position
        //   config);
    
        RamseteCommand ramseteCommand =
        new RamseteCommand(
          trajectory,
          DriveSubsystem.getInstance()::getPose,
          new RamseteController(Constants.DriveTrain.AUTO_B, Constants.DriveTrain.AUTO_ZETA),
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          DriveSubsystem.getInstance().getKinematics(),
          DriveSubsystem.getInstance()::getWheelSpeeds,
          new PIDController(Constants.KP, Constants.KI, Constants.KD),
          new PIDController(Constants.KP, Constants.KI, Constants.KD),
          DriveSubsystem.getInstance()::setVoltage,
          DriveSubsystem.getInstance());
    
          DriveSubsystem.getInstance().resetOdometry(trajectory.getInitialPose());
    
          return ramseteCommand.andThen(() -> DriveSubsystem.getInstance().setVoltage(0, 0));
    
      }
}
