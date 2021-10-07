package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DiagnosticCommand;
import frc.robot.commands.VelocityTestCommand;
import frc.robot.commands.DriveCommand.DriveMode;

public class RobotContainer {


  public RobotContainer() {
    configureButtonBindings();
    SmartDashboard.putData(new DriveCommand(DriveMode.PERCENT));
    SmartDashboard.putData(new DiagnosticCommand());
    SmartDashboard.putData(new VelocityTestCommand());
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
