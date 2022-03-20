package frc.robot.lib;

public class ColorDetection {

    public enum BallColor {
        RED, BLUE, NONE
    }

    static final double[] RED = {0.185, 0.556, 0.013, 0.246};
    static final double[] BLUE = {0.462, 0.374, 0.008, 0.156};

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
        double dRed = distance(c, RED);
        double dBlue = distance(c, BLUE);
        if(dRed < .13 && dBlue < .115) return (dRed < dBlue) ? BallColor.RED : BallColor.BLUE;
        else if(dRed < .13) return BallColor.RED;
        else if(dBlue < .115) return BallColor.BLUE;
        else return BallColor.NONE;
    }
}
