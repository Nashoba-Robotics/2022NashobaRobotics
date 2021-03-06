package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public static BallColor detect(double[] color) {
        double[] c = normalize(color);
        double dRed1 = distance(c, Constants.ColorSensor.RED1);
        double dRed2 = distance(c, Constants.ColorSensor.RED2);
        double dRed = Math.min(dRed1, dRed2);
        double dBlue1 = distance(c, Constants.ColorSensor.BLUE1);
        double dBlue2 = distance(c, Constants.ColorSensor.BLUE2);
        double dBlue = Math.min(dBlue1, dBlue2);
        SmartDashboard.putNumber("dRed", dRed);
        SmartDashboard.putNumber("dBlue", dBlue);
        if(dRed < Constants.ColorSensor.D_RED_MAX && dBlue < Constants.ColorSensor.D_BLUE_MAX) return (dRed < dBlue) ? BallColor.RED : BallColor.BLUE;
        else if(dRed < Constants.ColorSensor.D_RED_MAX) return BallColor.RED;
        else if(dBlue < Constants.ColorSensor.D_BLUE_MAX) return BallColor.BLUE;
        else return BallColor.NONE;
    }
}
