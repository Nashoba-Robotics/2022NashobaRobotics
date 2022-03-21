/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Currency;

import javax.sound.midi.SysexMessage;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Limelight;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutoStopIntakeCommand;
import frc.robot.commands.IntakePracticeCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.autoroutines.TestPathCommand;
import frc.robot.commands.autoroutines.ThreeBallAuto;
import frc.robot.commands.autoroutines.TwoBallAuto;
import frc.robot.lib.PicoColorSensor;
import frc.robot.lib.PicoColorSensor.RawColor;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  private PowerDistribution pdh;
  Compressor compressor;
  PneumaticHub ph = new PneumaticHub();
  SendableChooser<Command> autoChooser;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    pdh = new PowerDistribution();
    pdh.setSwitchableChannel(true);

    LimelightSubsystem.getInstance().setShooterLed(1);
    LimelightSubsystem.getInstance().setIntakeLed(1);

    // compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    // compressor.enableDigital();
    //ph.enableCompressorAnalog(100, 120);
    ph.disableCompressor();
    CommandScheduler.getInstance().setDefaultCommand(DriveSubsystem.getInstance(), new JoystickDriveCommand());

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Two Ball Auto", new TwoBallAuto());
    autoChooser.addOption("Three Ball Auto", new ThreeBallAuto());
    autoChooser.addOption("Test Auto", new TestPathCommand());
    SmartDashboard.putData("Auto", autoChooser);    
  }

  long lastMillis = System.currentTimeMillis();

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want run during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    long millis = System.currentTimeMillis();
    long diff = millis - lastMillis;
    lastMillis = millis;

    SmartDashboard.putNumber("Loop time", diff);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {  //Ensures that everything is stopped and is in an "off" state
    DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
    DriveSubsystem.getInstance().setSpeed(0, 0);
    IntakeSubsystem.getInstance().stop();
    IntakeSolenoidSubsystem.getInstance().retract();  //Undeploys the intake when the robot is disabled
    CannonSubsystem.getInstance().setAngle(Angle.EIGHTY);
    DriveSubsystem.getInstance().changeNeutralMode(NeutralMode.Coast);  //Sets the robot into "coast" mode after robot is diabled -> Easier to move
    LimelightSubsystem.getInstance().setIntakeLed(1);
    LimelightSubsystem.getInstance().setShooterLed(1);
    ClimberSubsystem.getInstance().stop();
    //Cancels everything that's running
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    LimelightSubsystem.getInstance().setIntakeLed(3);
    LimelightSubsystem.getInstance().setShooterLed(3);
    DriveSubsystem.getInstance().resetOdometryTrue();
    DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));DriveSubsystem.getInstance().resetOdometryTrue();
    DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));DriveSubsystem.getInstance().resetOdometryTrue();
    DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    autoChooser.getSelected().schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */

  //Iterates through an Auto Path array
  //If the instruction starts with "Path", then it will follow a path
  //Otherwise it will run the specified command
  @Override
  public void autonomousPeriodic() {
    
  }
  
  
  //Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CommandScheduler.getInstance().cancelAll();

    LimelightSubsystem.getInstance().setIntakeLed(3);
    LimelightSubsystem.getInstance().setShooterLed(3);

    DriveSubsystem.getInstance().resetOdometryTrue();
    DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    //Zeroes the climbers when teleop starts
    //CommandScheduler.getInstance().schedule(new ZeroClimberCommand());
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopPeriodic() {
    // RawColor c1 = ColorSensorSubsystem.getInstance().getSensor1();
    // SmartDashboard.putNumber("red1", 1.*c1.red/c1.ir);
    // SmartDashboard.putNumber("green1", 1.*c1.green/c1.ir);
    // SmartDashboard.putNumber("blue1", 1.*c1.blue/c1.ir);
    // RawColor c2 = ColorSensorSubsystem.getInstance().getSensor2();
    // SmartDashboard.putNumber("red2", 1.*c2.red/c2.ir);
    // SmartDashboard.putNumber("green2", 1.*c2.green/c2.ir);
    // SmartDashboard.putNumber("blue2", 1.*c2.blue/c2.ir);

    if(RobotContainer.getSensor1() && RobotContainer.getSensor2()){
      LedSubsystem.getInstance().setColor(255, 0, 0);
    } else if(RobotContainer.getSensor1() || RobotContainer.getSensor2()){
      LedSubsystem.getInstance().setColor(0, 0, 255);
    } else{
      LedSubsystem.getInstance().setColor(255, 255, 255);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().setDefaultCommand(DriveSubsystem.getInstance(), new JoystickDriveCommand());
    CommandScheduler.getInstance().schedule(new IntakePracticeCommand());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
