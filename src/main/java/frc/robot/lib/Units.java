package frc.robot.lib;

import frc.robot.Constants;

public class Units {
    // Convert from percent output to velocity (motor units / 100ms)
    public static double percent2Velocity(double percent) {
        return percent*Constants.VELOCITY_MULTIPLIER;
    }

    //converts from velocity (motor units / 100ms) to percent output
    public static double velocity2Percent(double velocity){
        return velocity/Constants.VELOCITY_MULTIPLIER;
    }

    //will round num to decimal places
    public static double roundTo(double num, int places) {
        return Math.round(num * Math.pow(10, places)) * Math.pow(10, -places);
    }

    public static double inches2Meters(double num){
        return num * 0.0254;
    }
}