package frc.robot.lib;

import frc.robot.Constants;

public class ColorDetection {

    public enum BallColor {
        RED, BLUE, NONE
    }

    private static double distance(double[] a, double[] b) {
        double sum = 0;
        for(int i = 0; i < a.length; i++) {
            sum += (a[i] - b[i])*(a[i] - b[i]);
        }
        return Math.sqrt(sum);
    }

    public static double[] normalize(double[] arr) {
        double total = arr[0] + arr[1] + arr[2] + arr[3];
        return new double[]{arr[0]/total, arr[1]/total, arr[2]/total, arr[3]/total};
    }

    public static BallColor detect(double[] c) {
        double dRed = distance(c, Constants.ColorSensor.RED);
        double dBlue = distance(c, Constants.ColorSensor.BLUE);
        if(dRed < Constants.ColorSensor.D_RED_MAX && dBlue < Constants.ColorSensor.D_BLUE_MAX) return (dRed < dBlue) ? BallColor.RED : BallColor.BLUE;
        else if(dRed < Constants.ColorSensor.D_RED_MAX) return BallColor.RED;
        else if(dBlue < Constants.ColorSensor.D_BLUE_MAX) return BallColor.BLUE;
        else return BallColor.NONE;
    }
}
