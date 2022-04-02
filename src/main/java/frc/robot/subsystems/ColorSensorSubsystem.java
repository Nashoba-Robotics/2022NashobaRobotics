package frc.robot.subsystems;

import frc.robot.lib.ColorDetection;
import frc.robot.lib.PicoColorSensor;
import frc.robot.lib.ColorDetection.BallColor;
import frc.robot.lib.PicoColorSensor.RawColor;

public class ColorSensorSubsystem {

    private PicoColorSensor sensors;

    private static ColorSensorSubsystem instance;
    public static ColorSensorSubsystem getInstance() {
        if(instance == null) {
            instance = new ColorSensorSubsystem();
        }
        return instance;
    }

    public ColorSensorSubsystem() {
        sensors = new PicoColorSensor();
    }

    public RawColor getSensor1() {
        return sensors.getRawColor0();
    }

    public BallColor getBall() {
        double[] color1 = new double[4];
        double[] color2 = new double[4];
        if(sensors.isSensor0Connected() && sensors.isSensor1Connected()) {
            RawColor s1 = getSensor1();
            RawColor s2 = getSensor2();
            color1 = new double[]{s1.red, s1.green, s1.blue, s1.ir};
            color2 = new double[]{s2.red, s2.green, s2.blue, s2.ir};
            // color[0] = (s1.red + s2.red)/2.0;
            // color[1] = (s1.green + s2.green)/2.0;
            // color[2] = (s1.blue + s2.blue)/2.0;
            // color[3] = (s1.ir + s2.ir)/2.0;
        } else if(sensors.isSensor0Connected() || sensors.isSensor1Connected()) {
            RawColor rc;
            if(sensors.isSensor0Connected()) rc = getSensor1();
            else rc = getSensor2();
            color1 = new double[]{rc.red, rc.green, rc.blue, rc.ir};
        } else {
            return BallColor.NONE;
        }
        BallColor c1 = ColorDetection.detect(color1);
        BallColor c2 = ColorDetection.detect(color2);
        if(c1 == BallColor.RED && c2 == BallColor.BLUE || c1 == BallColor.BLUE && c2 == BallColor.RED) return BallColor.NONE;
        if(c1 == BallColor.RED || c2 == BallColor.RED) return BallColor.RED;
        else if(c1 == BallColor.BLUE || c2 == BallColor.BLUE) return BallColor.BLUE;
        return BallColor.NONE;
    }



 /*   public BallColor getSensor1Color(){
        if(getSensor1().blue >= Constants.ColorSensor.BLUE_CONSTANT
        && getSensor1().red < Constants.ColorSensor.RED_CONSTANT) return BallColor.BLUE;
        else if(getSensor1().red >= Constants.ColorSensor.RED_CONSTANT
        && getSensor1().blue < Constants.ColorSensor.BLUE_CONSTANT) return BallColor.RED;
        else return BallColor.NONE;

        
    }

    public BallColor getSensor2Color(){
        if(getSensor1().blue >= Constants.ColorSensor.BLUE_CONSTANT
        && getSensor1().red < Constants.ColorSensor.RED_CONSTANT) return BallColor.BLUE;
        else if(getSensor1().red >= Constants.ColorSensor.RED_CONSTANT
        && getSensor1().blue < Constants.ColorSensor.BLUE_CONSTANT) return BallColor.RED;
        else return BallColor.NONE;
    } */

    // public BallColor getSensorColor(){
    //     if((getSensor1().red + getSensor1().green + getSensor1().blue) >= (getSensor2().red + getSensor2().green + getSensor2().blue)){
    //         if(getSensor1().blue >= Constants.ColorSensor.BLUE_CONSTANT
    //         && getSensor1().red < Constants.ColorSensor.RED_CONSTANT) return BallColor.BLUE;
    //         else if(getSensor1().red >= Constants.ColorSensor.RED_CONSTANT
    //         && getSensor1().blue < Constants.ColorSensor.BLUE_CONSTANT) return BallColor.RED;
    //         else return BallColor.NONE;
    //     }else{
    //         if(getSensor2().blue >= Constants.ColorSensor.BLUE_CONSTANT
    //         && getSensor2().red < Constants.ColorSensor.RED_CONSTANT) return BallColor.BLUE;
    //         else if(getSensor2().red >= Constants.ColorSensor.RED_CONSTANT
    //         && getSensor2().blue < Constants.ColorSensor.BLUE_CONSTANT) return BallColor.RED;
    //         else return BallColor.NONE;
    //     }
        
    // }

    public RawColor getSensor2() {
        return sensors.getRawColor1();
    }
    
    
}
