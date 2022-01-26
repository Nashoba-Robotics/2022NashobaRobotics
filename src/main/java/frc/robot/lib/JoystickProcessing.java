package frc.robot.lib;

import frc.robot.Constants;

//Processes joysticks
public class JoystickProcessing {

    public static double scaleJoystick(double value, double deadzone) {
        if(Math.abs(value) < deadzone) {
            return 0;
        } else {
            return Math.signum(value) * (Math.abs(value) - deadzone) / (1 - deadzone);
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
        return Math.pow(Math.abs(value), sensetivity) * Math.signum(value);
    }

    // Apply shaping to the turning and movement inputs
    public static JoystickValues shapeJoysticks(JoystickValues joystickValues) {
        double movementShaped = shapeJoystick(joystickValues.movement, Constants.MOVEMENT_SENSITIVITY);
        double turningShaped = shapeJoystick(joystickValues.turning, Constants.ARCADE_TURNING_SENSITIVITY);
        return new JoystickValues(movementShaped, turningShaped);
    }

    public static JoystickValues shapeJoysticksRadiusDrive(JoystickValues joystickValues) {
        double movementShaped = shapeJoystick(joystickValues.movement, Constants.MOVEMENT_SENSITIVITY);
        double turningShaped = shapeJoystick(joystickValues.turning, Constants.RADIUS_TURNING_SENSITIVITY);
        return new JoystickValues(movementShaped, turningShaped);
    }

    // Combine the movement and turing values to calculate
    // the left and right motor speeds
    public static MotorValues arcadeDrive(JoystickValues joystickValues) {
        // if(joystickValues.movement < 0) {
        //     joystickValues.turning *= -1;
        // }

        double left = joystickValues.movement + joystickValues.turning;
        double right = joystickValues.movement - joystickValues.turning;
        // If either side is above 1, divide by the max amount
        // to limit the maximum speed to 1 while preserving turn angle
        if(Math.abs(left) > 1 || Math.abs(right) > 1) {
            double factor = Math.max(Math.abs(left), Math.abs(right));
            left /= factor;
            right /= factor;
        }
        return new MotorValues(left, right);
    }

    public static MotorValues radiusDrive(JoystickValues joystickValues) {
        double left, right;
        joystickValues.turning *= Constants.RADIUS_DRIVE_MULTIPLIER;
        joystickValues.movement = Math.signum(joystickValues.movement)*
        Math.min(Math.abs(joystickValues.movement), allowedTipVelocity(Math.abs(joystickValues.turning)));
        double turning = joystickValues.turning*Math.abs(joystickValues.movement);
        if(joystickValues.movement > 0) {
            left = joystickValues.movement + turning;
            right = joystickValues.movement - turning;
        } else {
            left = joystickValues.movement - turning;
            right = joystickValues.movement + turning;
        }
        if(Math.abs(left) > 1 || Math.abs(right) > 1) {
            double factor = Math.max(Math.abs(left), Math.abs(right));
            left /= factor;
            right /= factor;
        }
        return new MotorValues(left, right);
    }

    //returns the max allowed velocity in order to prevent tipping
    private static double allowedTipVelocity(double turn) {
        double k_turn = Constants.WIDTH/2;
        double sqrtNumerator = Constants.GRAVITY * (Constants.WIDTH/2) * k_turn;
        double sqrtDenominator = 4 * Constants.HEIGHT * turn;
        return Math.sqrt(sqrtNumerator/sqrtDenominator)/Constants.METRICMAX;
    }

    // Apply all joystick processing
    public static MotorValues processJoysticksArcadeDrive(JoystickValues joystickValues) {
        JoystickValues scaledJoysticks = scaleJoysticks(joystickValues);
        JoystickValues shapedJoysticks = shapeJoysticks(scaledJoysticks);
        MotorValues motorSpeeds = arcadeDrive(shapedJoysticks);
        return motorSpeeds;
    }
    public static MotorValues processJoysticksRadiusDrive(JoystickValues joystickValues) {
        JoystickValues scaledJoysticks = scaleJoysticks(joystickValues);
        JoystickValues shapedJoysticks = shapeJoysticksRadiusDrive(scaledJoysticks);
        MotorValues motorSpeeds = radiusDrive(shapedJoysticks);
        return motorSpeeds;
    }
}