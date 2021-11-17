package frc.robot.lib;

import frc.robot.Constants;

public class Units {
    // Convert from percent output to velocity (motor units / 100ms)
    public static double percent2Velocity(double percent) {
        return percent*Constants.VELOCITY_MULTIPLIER;
    }
}