package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.VelocityTestCommand;

public class RobotContainer {


  public RobotContainer() {
    configureButtonBindings();
    SmartDashboard.putData(new JoystickDriveCommand());
    SmartDashboard.putData(new VelocityTestCommand());
    SmartDashboard.putData(new StopCommand());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


}
