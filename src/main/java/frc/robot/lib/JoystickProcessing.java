package frc.robot.lib;

import frc.robot.Constants;

public class JoystickProcessing {
    // Process the joystick value to make the controls easier to use
    // Equation: 
    // TODO separate into scaling and shaping functions
    public static double shapeJoystick(double value, double sensetivity) {
        if(Math.abs(value) < Constants.DEAD_ZONE) {
            return 0;
        }
        double scaledValue = (Math.abs(value) - Constants.DEAD_ZONE) / (1 - Constants.DEAD_ZONE);
        return Math.pow(scaledValue, sensetivity) * Math.signum(value);
    }

    // Apply shaping to the turning and movement inputs
    public static double[] shapeJoysticks(double rawMove, double rawTurn) {
        double movementShaped = shapeJoystick(rawMove, Constants.MOVEMENT_SENSITIVITY);
        double turningShaped = shapeJoystick(rawTurn, Constants.TURN_SENSITIVITY);
        return new double[]{movementShaped, turningShaped};
    }

    // Combine the movement and turing values to calculate
    // the left and right motor speeds
    public static double[] arcadeDrive(double movement, double turning) {
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
    // TODO move raw joystick polling to this file
    public static double[] processJoysticks(double rawMove, double rawTurn) {
        double[] shaped = shapeJoysticks(rawMove, rawTurn);
        double[] motorSpeeds = arcadeDrive(shaped[0], shaped[1]);
        return motorSpeeds;        
    }
}