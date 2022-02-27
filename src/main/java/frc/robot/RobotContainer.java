package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HybridDriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.LoaderCommand;
import frc.robot.commands.SensorTestCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.TalonTestCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.VelocityTestCommand;
import frc.robot.subsystems.Drive2019Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.commands.GyroTestCommand;
import frc.robot.commands.CannonTestCommand;
import frc.robot.commands.ClimberCommand;

public class RobotContainer {

  public static JoystickDriveCommand joystickDriveCommand = new JoystickDriveCommand();
  public static HybridDriveCommand hybridDriveCommand = new HybridDriveCommand();

  public RobotContainer() {
    configureButtonBindings();
    // SmartDashboard.putData(joystickDriveCommand);
    // SmartDashboard.putData(new VelocityTestCommand());
    SmartDashboard.putData(new StopCommand());
    // SmartDashboard.putData(new SensorTestCommand());
    // SmartDashboard.putData(hybridDriveCommand);
    // SmartDashboard.putData(new AutoDriveCommand());
    // SmartDashboard.putData(new TalonTestCommand());
    //SmartDashboard.putData(new TurretCommand());
    // SmartDashboard.putData(new GyroTestCommand());
    // SmartDashboard.putData(new LimelightCommand());
    //SmartDashboard.putData(new CannonTestCommand());
    SmartDashboard.putData(new ClimberCommand());
    //SmartDashboard.putData(new LoaderCommand());
    // SmartDashboard.putData(new TalonTestCommand());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 0);
  }

  public Command getAutonomousCommand(){
    DifferentialDriveVoltageConstraint autoVoltageConstraint = 
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
      DriveSubsystem.getInstance().getKinematics(),
      8);

    TrajectoryConfig config =
    new TrajectoryConfig(
      Constants.DriveTrain.MAX_VELOCITY,
      Constants.DriveTrain.MAX_ACCELERATION)
      .setKinematics(DriveSubsystem.getInstance().getKinematics())
      .addConstraint(autoVoltageConstraint);

    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), //starting position
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
      new Pose2d(3, 0, new Rotation2d(0)),
      config);

    RamseteCommand ramseteCommand =
    new RamseteCommand(
      trajectory,
      DriveSubsystem.getInstance()::getPose,
      new RamseteController(),
      new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
      DriveSubsystem.getInstance().getKinematics(),
      DriveSubsystem.getInstance()::getWheelSpeeds,
      new PIDController(0, 0, 0.8),
      new PIDController(0, 0, 0.8),
      DriveSubsystem.getInstance()::setVoltage,
      DriveSubsystem.getInstance());

      DriveSubsystem.getInstance().resetOdometry(trajectory.getInitialPose());

      return ramseteCommand.andThen(() -> DriveSubsystem.getInstance().setVoltage(0, 0));

  }

}
