package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import com.fasterxml.jackson.core.JsonParser;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakePracticeCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.LedTestCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.LoaderCommand;
import frc.robot.commands.ManualZeroOdometryCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.SensorTestCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.climber.DeployPusher;
import frc.robot.commands.climber.DeployClimberCommad;
import frc.robot.commands.climber.ManualPusherCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.TraversalClimbCommand;
import frc.robot.commands.climber.ZeroClimberSensorsCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.EjectBackCommand;
import frc.robot.commands.intakeshoot.EjectFrontCommand;
import frc.robot.commands.intakeshoot.PukeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.commands.intakeshoot.ShootCommand;
import frc.robot.commands.intakeshoot.StopIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;
import frc.robot.commands.CannonTestCommand;
import frc.robot.commands.ColorSensorTestCommand;
import frc.robot.commands.DiagnosticClimberCommand;

public class RobotContainer {

  public static JoystickDriveCommand joystickDriveCommand = new JoystickDriveCommand();

  //Initializes all buttons used for commands
  //Takes in a joystick and the button poer
  //IMPORTANT!!: Joystick Buttons are 1 indexed (They start at 1 instead of 0)
  Trigger deployIntakeSwitch = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.DEPLOY_INTAKE);

  Trigger runIntakeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.RUN_INTAKE).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger stopIntakeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.STOP_INTAKE).debounce(Constants.Buttons.DEBOUNCE_VALUE);
                                                                                                                                  
  Trigger ejectFrontButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.EJECT_FRONT).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger ejectBackButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.EJECT_BACK).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger pukeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.PUKE).debounce(Constants.Buttons.DEBOUNCE_VALUE);

  Trigger shootButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.SHOOT).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger runShooterButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.RUN_SHOOTER).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger stopShooterButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.STOP_SHOOTER);
  Trigger shooterAngleSwitch = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.SHOOTER_ANGLE).debounce(Constants.Buttons.DEBOUNCE_VALUE);

  Trigger fixedClimbDeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.FIXED_CLIMB_DEPLOY).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger fixedClimbButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.FIXED_CLIMB).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger fixedClimbeGrabButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.FIXED_CLIMB_GRAB).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  Trigger fixedClimberReleaseButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.FIXED_CLIMB_RELEASE).debounce(Constants.Buttons.DEBOUNCE_VALUE);

  // Trigger rotatingClimbDeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 6).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  // Trigger rotatingClimbUndeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 5).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  // Trigger rotatingClimbeGrabButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 4).debounce(Constants.Buttons.DEBOUNCE_VALUE);
  // Trigger rotatingClimberReleaseButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 5).debounce(Constants.Buttons.DEBOUNCE_VALUE);

  static DigitalInput ballSensor1 = new DigitalInput(Constants.Intake.DIO_SENSOR_1);
  static DigitalInput ballSensor2 = new DigitalInput(Constants.Intake.DIO_SENSOR_2);

  public static boolean getSensor1() {
    return !ballSensor1.get();
  }
  public static boolean getSensor2() {
    return !ballSensor2.get();
  }

  public RobotContainer() {
    configureButtonBindings();
    SmartDashboard.putData(joystickDriveCommand);
    // SmartDashboard.putData(new VelocityTestCommand());
    // SmartDashboard.putData(new StopCommand());
    // SmartDashboard.putData(new SensorTestCommand());
    // SmartDashboard.putData(hybridDriveCommand);
    // SmartDashboard.putData(new AutoDriveCommand());
    // SmartDashboard.putData(new TalonTestCommand());
    // SmartDashboard.putData(new TurretCommand());
    // SmartDashboard.putData(new GyroTestCommand());
    // SmartDashboard.putData(new LimelightCommand());
    // SmartDashboard.putData(new CannonTestCommand());
    SmartDashboard.putData(new ClimberCommand());
    SmartDashboard.putData(new ZeroClimberSensorsCommand());
    SmartDashboard.putData(new DeployClimberCommad());
    SmartDashboard.putData(new RetractClimberCommand());
    SmartDashboard.putData(new ManualPusherCommand());
    SmartDashboard.putData(new DeployPusher());
    SmartDashboard.putData(new TraversalClimbCommand());
    //SmartDashboard.putData(new DiagnosticClimberCommand());
    // SmartDashboard.putData(new ZeroClimberCommand());
    // SmartDashboard.putData(new LoaderCommand());
    // SmartDashboard.putData(new TalonTestCommand());
    // SmartDashboard.putData(new ActuateIntakeCommand(true));
    // SmartDashboard.putData(new ActuateIntakeCommand(false));
    // SmartDashboard.putData(new EjectFrontCommand());
    // SmartDashboard.putData(new EjectBackCommand());
    // SmartDashboard.putData(new PukeCommand());
    // SmartDashboard.putData(new RunIntakeCommand());
    // SmartDashboard.putData(new ShootCommand());
    // SmartDashboard.putData(new LoaderCommand());
    // SmartDashboard.putData(new CannonTestCommand());
    // SmartDashboard.putData(new IntakePracticeCommand());
    //SmartDashboard.putData(new AutoAimCommand());
    // SmartDashboard.putData(new ManualZeroOdometryCommand());
    // SmartDashboard.putData(new LedTestCommand());
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    deployIntakeSwitch.whenInactive(new ActuateIntakeCommand(true));
    deployIntakeSwitch.whenActive(new ActuateIntakeCommand(false));


    runIntakeButton.whenActive(new RunIntakeCommand());
    stopIntakeButton.whenActive(new StopIntakeCommand());

    shooterAngleSwitch.whenActive(new CannonAngleCommand(Angle.EIGHTY));
    shooterAngleSwitch.whenInactive(new CannonAngleCommand(Angle.SIXTY));

    ejectFrontButton.whenActive(new EjectFrontCommand());
    ejectBackButton.whenActive(new EjectBackCommand());
    pukeButton.whenActive(new PukeCommand());

    ShootCommand shootCommand = new ShootCommand();
    runShooterButton.whenActive(shootCommand);
    stopShooterButton.cancelWhenActive(shootCommand);
    // runShooterButton.whenActive(new StopCommand()); //TODO: Change to actual Run Shooter Command

    fixedClimbDeployButton.whenActive(new DeployClimberCommad());
    fixedClimbButton.whenActive(new RetractClimberCommand());
    // fixedClimbeGrabButton.whenActive(new StopCommand());  //TODO: Change to actual Grab Command
    // fixedClimberReleaseButton.whenActive();

    // rotatingClimbDeployButton.whenActive(new StopCommand());  //TODO: Change to actual Rotating Deploy Command
    // rotatingClimbUndeployButton.whenActive(new StopCommand());  //TODO: Change to actual Rotating Undeploy Command
    // rotatingClimbeGrabButton.whenActive(new StopCommand());   //TODO: Change to actual Rotating Grab Command
    // rotatingClimberReleaseButton.whenActive();
  }

}
