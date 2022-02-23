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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HybridDriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.SensorTestCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.TalonTestCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.VelocityTestCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.commands.GyroTestCommand;
import frc.robot.commands.CannonTestCommand;

public class RobotContainer {

  public static JoystickDriveCommand joystickDriveCommand = new JoystickDriveCommand();
  public static HybridDriveCommand hybridDriveCommand = new HybridDriveCommand();

  public RobotContainer() {
    configureButtonBindings();
    SmartDashboard.putData(joystickDriveCommand);
    SmartDashboard.putData(new VelocityTestCommand());
    SmartDashboard.putData(new StopCommand());
    SmartDashboard.putData(new SensorTestCommand());
    SmartDashboard.putData(hybridDriveCommand);
    SmartDashboard.putData(new AutoDriveCommand());
    // SmartDashboard.putData(new TalonTestCommand());
    //SmartDashboard.putData(new TurretCommand());
    SmartDashboard.putData(new GyroTestCommand());
    SmartDashboard.putData(new LimelightCommand());
    SmartDashboard.putData(new CannonTestCommand());
    // SmartDashboard.putData(Drive2019Subsystem.getInstance());
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

}
