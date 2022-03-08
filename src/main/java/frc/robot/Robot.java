/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShoot60Command;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutoStopIntakeCommand;
import frc.robot.commands.DeployStaticClimberCommad;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.RetractStaticClimberCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.ZeroClimberCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.DeployIntakeCommand;
import frc.robot.commands.intakeshoot.EjectBackCommand;
import frc.robot.commands.intakeshoot.EjectFrontCommand;
import frc.robot.commands.intakeshoot.PukeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.commands.intakeshoot.ShootCommand;
import frc.robot.commands.intakeshoot.StopIntakeCommand;
import frc.robot.commands.intakeshoot.RetractIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;


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

  private Command currCommand;
  private int currCommandIndex = 0;
  private boolean autoFinished = false;
  private String[] auto;

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
    // compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    // compressor.enableDigital();
    ph.enableCompressorAnalog(100, 120);
    CommandScheduler.getInstance().setDefaultCommand(DriveSubsystem.getInstance(), new JoystickDriveCommand());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
    //CommandScheduler.getInstance().setDefaultCommand(AbstractDriveSubsystem.getInstance(), new JoystickDriveCommand());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
    DriveSubsystem.getInstance().setSpeed(0, 0);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autoFinished = false;
    currCommandIndex = 0;

    auto = Constants.DriveTrain.AUTONOMOUS_ROUTINE; //Change here to switch the auto routine
    
    Command autoCommand = DriveSubsystem.getInstance().getAutonomousCommand("");
    autoCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */


  @Override
  public void autonomousPeriodic() {
    // if(!autoFinished
    // && (currCommand == null 
    // || !CommandScheduler.getInstance().isScheduled(currCommand)
    // && currCommandIndex < auto.length)){
    //   String[] parts = auto[currCommandIndex].split(" ");
    //   switch(parts[0]) {  
    //     case "path":
    //       currCommand = DriveSubsystem.getInstance().getAutonomousCommand(parts[1]);
    //       break;
    //     case "shoot":
    //       currCommand = new AutoShootCommand();
    //       break;
    //     case "intake":
    //       currCommand = new AutoIntakeCommand();
    //       break;
    //     case "stopIntake":
    //       currCommand = new AutoStopIntakeCommand();
    //       break;
    //     case "shoot60":
    //       currCommand = new AutoShoot60Command();
    //       break;
    //   } 
    //   currCommand.schedule();
    //   currCommandIndex++;
    // } else if(currCommandIndex >= auto.length){
    //   currCommandIndex = 0;
    //   autoFinished = true;
    // }
  }
  
  
  //Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CommandScheduler.getInstance().cancelAll();
    //compressor.enableDigital();
    CommandScheduler.getInstance().schedule(new ZeroClimberCommand());

    deployIntakeSwitch.whenInactive(new DeployIntakeCommand());
    deployIntakeSwitch.whenActive(new RetractIntakeCommand());


    runIntakeButton.whenActive(runIntakeCommand);
    stopIntakeButton.whenActive(new StopIntakeCommand());

    shooterAngleSwitch.whenActive(new CannonAngleCommand(true));
    shooterAngleSwitch.whenInactive(new CannonAngleCommand(false));

    ejectFrontButton.whenActive(new EjectFrontCommand());
    ejectBackButton.whenActive(new EjectBackCommand());
    pukeButton.whenActive(new PukeCommand());

    ShootCommand shootCommand = new ShootCommand();
    runShooterButton.whenActive(shootCommand);
    rotatingClimberReleaseButton.cancelWhenActive(shootCommand);
    // runShooterButton.whenActive(new StopCommand()); //TODO: Change to actual Run Shooter Command

    fixedClimbDeployButton.whenActive(new DeployStaticClimberCommad());
    fixedClimbUndeployButton.whenActive(new RetractStaticClimberCommand());
    // fixedClimbeGrabButton.whenActive(new StopCommand());  //TODO: Change to actual Grab Command
    // fixedClimberReleaseButton.whenActive();

    // rotatingClimbDeployButton.whenActive(new StopCommand());  //TODO: Change to actual Rotating Deploy Command
    // rotatingClimbUndeployButton.whenActive(new StopCommand());  //TODO: Change to actual Rotating Undeploy Command
    // rotatingClimbeGrabButton.whenActive(new StopCommand());   //TODO: Change to actual Rotating Grab Command
    // rotatingClimberReleaseButton.whenActive();
  }

  /**
   * This function is called periodically during operator control.
   */

  double smolValue = 0.1;
  Trigger deployIntakeSwitch = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 10);

  Trigger runIntakeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 1).debounce(smolValue); //Creates an instance of a button
  Trigger stopIntakeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 2).debounce(smolValue);//First parameter is the Joystick the button is on
  RunIntakeCommand runIntakeCommand = new RunIntakeCommand();      
                                                                                                                                  //Second parameter is the index of the Joystick
  Trigger ejectFrontButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 4).debounce(smolValue);//VERY IMPORTANT: Joysticks are 1-indexed, not 0-indexed
  Trigger ejectBackButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 6).debounce(smolValue);
  Trigger pukeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 3).debounce(smolValue);

  Trigger shootButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 9).debounce(smolValue);
  Trigger runShooterButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 8).debounce(smolValue);
  Trigger shooterAngleSwitch = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 7).debounce(smolValue);

  Trigger fixedClimbDeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 3).debounce(smolValue);
  Trigger fixedClimbUndeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 2).debounce(smolValue);
  Trigger fixedClimbeGrabButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 1).debounce(smolValue);
  Trigger fixedClimberReleaseButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 11).debounce(smolValue);

  Trigger rotatingClimbDeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 6).debounce(smolValue);
  Trigger rotatingClimbUndeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 5).debounce(smolValue);
  Trigger rotatingClimbeGrabButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 4).debounce(smolValue);
  Trigger rotatingClimberReleaseButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 5).debounce(smolValue);



  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
