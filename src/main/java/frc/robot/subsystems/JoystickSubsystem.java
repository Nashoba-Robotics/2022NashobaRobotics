package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import frc.robot.Constants;

// Subsystem for processing joystick inputs
public class JoystickSubsystem extends SubsystemBase {
    // The singleton instance; generated on the first call of getInstance()
    private static JoystickSubsystem instance;

    private Joystick leftJoystick;
    private Joystick rightJoystick;

    private JoystickSubsystem() {
        leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
        rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
        // Set the name of the subsystem in smart dashboard
        SendableRegistry.setName(this, "Joystick");
    }

    public static JoystickSubsystem getInstance() {
        if(instance == null) {
            instance = new JoystickSubsystem();
        }
        return instance;
    }

    // Get the raw X or Y position for either the left or right joystick
    public double getLeftX() {
        return leftJoystick.getX();
    }
    public double getLeftY() {
        return leftJoystick.getY();
    }
    public double getRightX() {
        return rightJoystick.getX();
    }
    public double getRightY() {
        return rightJoystick.getY();
    }
}
