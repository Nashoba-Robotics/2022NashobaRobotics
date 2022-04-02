package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.util.sendable.SendableRegistry;

import frc.robot.Constants;

// Subsystem for processing joystick inputs
public class JoystickSubsystem extends SubsystemBase {
    private static JoystickSubsystem instance;

    private Joystick leftJoystick;
    private Joystick rightJoystick;
    private Joystick leftOperatorJoystick;
    private Joystick rightOperatorJoystick;

    private JoystickSubsystem() {
        leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
        rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
        leftOperatorJoystick = new Joystick(Constants.LEFT_OPERATOR_JOYSTICK_PORT);
        rightOperatorJoystick = new Joystick(Constants.RIGHT_OPERATOR_JOYSTICK_PORT);
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
        return -leftJoystick.getY(); //Has to be negative when joystick inputs are reveresed
    }
    public double getRightX() {
        return rightJoystick.getX();
    }
    public double getRightY() {
        return -rightJoystick.getY(); //Has to be negative when joystick inputs are reveresed
    }
    public Joystick getLeftJoystick(){  //getLeft/RightJoystick for Robot Container button binding
        return leftJoystick;
    }
    public Joystick getRightJoystick(){
        return rightJoystick;
    }
    public Joystick getLeftOperatorJoystick(){
        return leftOperatorJoystick;
    }
    public Joystick getRightOperatorJoystick(){
        return rightOperatorJoystick;
    }
}
