package frc.robot.lib;

import frc.robot.Constants;

public class JoystickProcessing {
    // Process the joystick value to make the controls easier to use
    // Equation: 
    public static double shapeJoystick(double value, double sensetivity) {
        if(Math.abs(value) < Constants.DEAD_ZONE) {
            return 0;
        }
        double value2 = (Math.abs(value) - Constants.DEAD_ZONE) / (1 - Constants.DEAD_ZONE);
        return Math.pow(value2, sensetivity) * Math.signum(value);
    }

    // public static double turnShape(double value) {
    //     double s = Math.signum(value)*Math.pow(Math.abs(value), Constants.TURN_SENSITIVITY);
    //     if(Math.abs(s) < 0.05) {
    //         return 0;
    //     } else {
    //         return s;
    //     }
    // }

    // Apply shaping to the turning and movement inputs
    public static double[] shapeJoysticks(double movement, double turning) {
        double movementShaped = shapeJoystick(movement, Constants.MOVEMENT_SENSITIVITY);
        double turningShaped = shapeJoystick(turning, Constants.TURN_SENSITIVITY);
        return new double[]{movementShaped, turningShaped};
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