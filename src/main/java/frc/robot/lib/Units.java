package frc.robot.lib;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.lib.ColorDetection.BallColor;

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

    public static double degrees2Radians(double degrees){
        return degrees * (Constants.TAU / 360);
    }

    public static double radians2Degrees(double radians){
        return radians * (360 / Constants.TAU);
    }

    // turns any angle (radians) into 0 -> TAU space
    public static double getAbsAngle(double ang){
        return ((ang % Constants.TAU) + Constants.TAU) % Constants.TAU;
        // return ang > 0 ?
        // ang % Constants.TAU:
        // Constants.TAU - (Math.abs(ang) % Constants.TAU);
    }

    public static BallColor alliance2BallColor(Alliance alliance){
        if(alliance == Alliance.Blue){
            return BallColor.BLUE;
        } else if(alliance == Alliance.Red){
            return BallColor.RED;
        }
        return BallColor.NONE;
    }

    public static BallColor oppositeBallColor(BallColor color){
        if(color == BallColor.BLUE){
            return BallColor.RED;
        } else if(color == BallColor.RED){
            return BallColor.BLUE;
        }
        return BallColor.NONE;
    }

    //for drivetrain
    public static double NU2Meters(double nu){
        double rate = (Constants.TAU * Constants.DriveTrain.WHEEL_RADIUS) / (Constants.DriveTrain.DRIVE_GEAR_RATIO * Constants.FALCON_NU);
        return nu * rate;
    }

    public static double meters2NUSpeed(double metersPerSecond){
        return (metersPerSecond * (Constants.DriveTrain.DRIVE_GEAR_RATIO * Constants.FALCON_NU) / (Constants.TAU * Constants.DriveTrain.WHEEL_RADIUS)) / 10;
    }

}