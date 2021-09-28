package frc.robot.lib;

import frc.robot.Constants;

public class JoystickProcessing {
    // Process the joystick Y value to make the controls more precise
    // Equation: result = sign(value)*abs(value)^SENSETIVITY
    public static double movementShape(double value) {
        return Math.signum(value)*Math.pow(Math.abs(value), Constants.MOVEMENT_SENSITIVITY);
    }
    public static double turnShape(double value) {
        return Math.signum(value)*Math.pow(Math.abs(value), Constants.TURN_SENSITIVITY);
    }

    // Returns a double[] of size two containing the left and right motor inputs
    public static double[] processJoysticks(double joystickY, double joystickX) {
        // Shape both the movement and rotation
        double shapedX = turnShape(joystickX);
        double shapedY = movementShape(joystickY);
        // Turning: add the rotation to one side and subtract from the other
        double left = shapedY - shapedX;
        double right = shapedY + shapedX;
        // If either side is above 1, divide both by the maximum
        // to limit the maximum speed to 1 while preserving turning
        if(Math.abs(left) > 1 || Math.abs(right) > 1) {
            double factor = Math.max(Math.abs(left), Math.abs(right));
            left /= factor;
            right /= factor;
        }
        return new double[]{left, right};
    }
}