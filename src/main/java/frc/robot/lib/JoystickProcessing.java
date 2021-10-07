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

    // Apply shaping to the turning and movement inputs
    public static double[] shapeJoysticks(double movement, double turning) {
        return new double[]{turnShape(turning), movementShape(movement)};
    }

    // Combine the movement and turing values to calculate
    // the left and right motor speeds
    public static double[] calculateMotorSpeeds(double movement, double turning) {
        double left = movement - turning;
        double right = movement + turning;
        // If either side is above 1, divide both by the maximum
        // to limit the maximum speed to 1 while preserving turning angle
        if(Math.abs(left) > 1 || Math.abs(right) > 1) {
            double factor = Math.max(Math.abs(left), Math.abs(right));
            left /= factor;
            right /= factor;
        }
        return new double[]{left, right};
    }

    // Apply all joystick processing
    public static double[] processJoysticks(double joystickY, double joystickX) {
        double[] shaped = shapeJoysticks(joystickY, joystickX);
        //double[] shaped = {joystickY, joystickX};
        double[] motorSpeeds = calculateMotorSpeeds(shaped[0], shaped[1]);
        return motorSpeeds;        
    }
}