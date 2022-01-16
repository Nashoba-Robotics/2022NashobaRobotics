package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HybridDriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.ButtonTestCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.MusicCommand;
import frc.robot.commands.OdeToJoy;
import frc.robot.commands.SensorTestCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.VelocityTestCommand;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.commands.GyroTestCommand;
import frc.robot.commands.CannonTestCommand;

public class RobotContainer {

  public RobotContainer() {
    configureButtonBindings();
    SmartDashboard.putData(new JoystickDriveCommand());
    SmartDashboard.putData(new VelocityTestCommand());
    SmartDashboard.putData(new StopCommand());
    SmartDashboard.putData(new SensorTestCommand());
    SmartDashboard.putData(new HybridDriveCommand());
    SmartDashboard.putData(new AutoDriveCommand());
    //SmartDashboard.putData(new ButtonTestCommand());
    //SmartDashboard.putData(new TurretCommand());
    //SmartDashboard.putData(new MusicCommand());
    //SmartDashboard.putData(new OdeToJoy());
    SmartDashboard.putData(new GyroTestCommand());
    SmartDashboard.putData(new LimelightCommand());
    SmartDashboard.putData(new CannonTestCommand());
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
