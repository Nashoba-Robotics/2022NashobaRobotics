package frc.robot.lib;

//used to represent joystick input using move (y axis) and turn (x axis)
public class JoystickValues{
    public double movement;
    public double turning;

    public JoystickValues(double movement, double turning){
        this.movement = movement;
        this.turning = turning;
    }
}