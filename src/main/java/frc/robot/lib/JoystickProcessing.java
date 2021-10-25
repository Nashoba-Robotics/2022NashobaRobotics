package frc.robot.lib;

import frc.robot.Constants;

public class JoystickProcessing {

    public static double scaleJoystick(double value, double deadzone) {
        if(Math.abs(value) < deadzone) {
            return 0;
        } else {
            return (Math.abs(value) - deadzone) / (1 - deadzone);
        }
    }

    // Apply scaling to the turning and movement inputs
    public static JoystickValues scaleJoysticks(JoystickValues joystickValues) {
        double movementScaled = scaleJoystick(joystickValues.movement, Constants.MOVEMENT_DEADZONE);
        double turningScaled = scaleJoystick(joystickValues.turning, Constants.TURNING_DEADZONE);
        return new JoystickValues(movementScaled, turningScaled);
    }

    // Process the joystick value to make the controls easier to use
    // Equation: 
    public static double shapeJoystick(double value, double sensetivity) {
        return Math.pow(value, sensetivity) * Math.signum(value);
    }

    // Apply shaping to the turning and movement inputs
    public static JoystickValues shapeJoysticks(JoystickValues joystickValues) {
        double movementShaped = shapeJoystick(joystickValues.movement, Constants.MOVEMENT_SENSITIVITY);
        double turningShaped = shapeJoystick(joystickValues.turning, Constants.TURNING_SENSITIVITY);
        return new JoystickValues(movementShaped, turningShaped);
    }

    // Combine the movement and turing values to calculate
    // the left and right motor speeds
    public static MotorValues arcadeDrive(JoystickValues joystickValues) {
        double left = joystickValues.movement - joystickValues.turning;
        double right = joystickValues.movement + joystickValues.turning;
        // If either side is above 1, divide both by the maximum
        // to limit the maximum speed to 1 while preserving turning angle
        if(Math.abs(left) > 1 || Math.abs(right) > 1) {
            double factor = Math.max(Math.abs(left), Math.abs(right));
            left /= factor;
            right /= factor;
        }
        return new MotorValues(left, right);
    }

    // Apply all joystick processing
    public static MotorValues processJoysticks(JoystickValues joystickValues) {
        JoystickValues scaledJoysticks = scaleJoysticks(joystickValues);
        JoystickValues shapedJoysticks = shapeJoysticks(scaledJoysticks);
        MotorValues motorSpeeds = arcadeDrive(shapedJoysticks);
        return motorSpeeds;        
    }
}