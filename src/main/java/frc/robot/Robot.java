/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShoot60Command;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutoStopIntakeCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.ZeroClimberCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {  //Ensures that everything is stopped and is in an "off" state
    DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
    DriveSubsystem.getInstance().setSpeed(0, 0);
    IntakeSubsystem.getInstance().stop();
    IntakeSubsystem.getInstance().retractIntake();  //Undeploys the intake when the robot is disabled
    DriveSubsystem.getInstance().changeNeutralMode(NeutralMode.Coast);  //Sets the robot into "coast" mode after robot is diabled -> Easier to move
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

    auto = Constants.DriveTrain.SIMPLE_AUTO; //Change here to switch the auto routine
  }

  /**
   * This function is called periodically during autonomous.
   */

  //Iterates through an Auto Path array
  //If the instruction starts with "Path", then it will follow a path
  //Otherwise it will run the specified command
  @Override
  public void autonomousPeriodic() {
    if(!autoFinished
    && (currCommand == null 
    || !CommandScheduler.getInstance().isScheduled(currCommand)
    && currCommandIndex < auto.length)){
      String[] parts = auto[currCommandIndex].split(" ");
      switch(parts[0]) {  
        case "path":
          currCommand = DriveSubsystem.getInstance().getAutonomousCommand(parts[1]);
          break;
        case "shoot":
          currCommand = new AutoShootCommand();
          break;
        case "intake":
          currCommand = new AutoIntakeCommand();
          break;
        case "stopIntake":
          currCommand = new AutoStopIntakeCommand();
          break;
        case "shoot60":
          currCommand = new AutoShoot60Command();
          break;
      } 
      currCommand.schedule();
      currCommandIndex++;
    } else if(currCommandIndex >= auto.length){
      currCommandIndex = 0;
      autoFinished = true;
    }
  }
  
  
  //Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CommandScheduler.getInstance().cancelAll();

    //Zeroes the climbers when teleop starts
    CommandScheduler.getInstance().schedule(new ZeroClimberCommand());
  }

  /**
   * This function is called periodically during operator control.
   */

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
