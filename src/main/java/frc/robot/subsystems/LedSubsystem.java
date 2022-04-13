package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private static final int LIGHT_COUNT = 38;

    private static LedSubsystem instance;
    public static LedSubsystem getInstance() {
        if(instance == null) {
            instance = new LedSubsystem();
        }
        return instance;
    }

    public enum LedStateType {
        NONE, AUTO, BALLS, BALLS_BLINK, CLIMB, FMS_DISABLE, GRACIOUS_PROFESSIONALISM
    }

    private CANdle candle;
    
    private LedStateType type = LedStateType.NONE;
    private int[] ballColor = {0,0,0};
    private int[] climbColor = {0,0,0};

    private LedSubsystem() {
        candle = new CANdle(0);
        candle.configFactoryDefault();
        candle.configLOSBehavior(true);
        candle.configStatusLedState(true);
        candle.configBrightnessScalar(0.7);
    }

    public void setLedStateType(LedStateType newType) {
        if(type != newType) {
            type = newType;
            updateLeds();
        }
    }

    public void setBallColor(int[] color) {
        boolean update = color[0] != ballColor[0] || color[1] != ballColor[1] || color[2] != ballColor[2];
        ballColor = color;
        if(update && (type == LedStateType.BALLS || type == LedStateType.BALLS_BLINK)) {
            updateLeds();
        }
    }

    public void setClimbColor(int[] color) {
        boolean update = color[0] != climbColor[0] || color[1] != climbColor[1] || color[2] != climbColor[2];
        climbColor = color;
        if(type == LedStateType.CLIMB && update) {
            updateLeds();
        }
    }

    private void updateLeds() {
        candle.setLEDs(ballColor[0], ballColor[1], ballColor[2]);
        System.out.println(ballColor[0] + " " + ballColor[1] + " " + ballColor[2]);
        switch(type) {
            case NONE:
                candle.clearAnimation(0);
                break;
            case AUTO:
                Animation a0 = new RainbowAnimation(1, 0.9, LIGHT_COUNT + 46);
                candle.animate(a0);
                break;
            case BALLS:
                Animation a1 = new StrobeAnimation(ballColor[0], ballColor[1], ballColor[2], 0, 1, 46, 0);
                candle.animate(a1);
                break;
            case BALLS_BLINK:
                Animation a2 = new StrobeAnimation(ballColor[0], ballColor[1], ballColor[2], 0, 0.2, 46, 0);
                candle.animate(a2);
                break;
            case FMS_DISABLE:
                Animation a3 = new StrobeAnimation(0, 128, 0, 0, 1, 46, 0);
                candle.animate(a3);
            default:
                break;
        }
        // System.out.println("LEDS UPDATED");
        // switch(type) {
        //     case NONE:
        //         candle.setLEDs(0, 0, 0, 0, 0, LIGHT_COUNT + 8);
        //         break;
        //     case AUTO:
        //         // Animation a1 = new RainbowAnimation(1, RAINBOW_SPEED, LIGHT_COUNT + 8);
        //         // candle.animate(a1);
        //         break;
        //     case BALLS:
        //         candle.setLEDs(ballColor[0], ballColor[1], ballColor[2], 0, 8, LIGHT_COUNT);
        //         break;
        //     case BALLS_BLINK:
        //         // Animation a2 = new LarsonAnimation(ballColor[0], ballColor[1], ballColor[2]);
        //         // //Animation a2 = new RainbowAnimation();
        //         // //Animation a2 = new StrobeAnimation(ballColor[0], ballColor[1], ballColor[2], 0, 0.5, LIGHT_COUNT + 8);
        //         // //Animation a2 = new TwinkleAnimation(ballColor[0], ballColor[1], ballColor[2], 0, 0.5, LIGHT_COUNT + 8, TwinklePercent.Percent76);//(ballColor[0], ballColor[1], ballColor[2], 0, 0.5, LIGHT_COUNT + 8);
        //         // candle.animate(a2);
        //         break;
        //     case CLIMB:
        //         // Animation a3 = new TwinkleAnimation(climbColor[0], climbColor[1], climbColor[2], 0, 0.5, LIGHT_COUNT + 8, TwinklePercent.Percent64);
        //         // candle.animate(a3);
        //         break;
        //     case FMS_DISABLE:
        //         // Animation a4 = new RgbFadeAnimation(1, 0.5, LIGHT_COUNT + 8);
        //         // candle.animate(a4);
        //         break;
        //     case GRACIOUS_PROFESSIONALISM:
        //         // Animation a5 = new FireAnimation(1, 0.75, LIGHT_COUNT + 8, 0.7, 0.4);
        //         // candle.animate(a5);
        //         break;
        // }
    }
}
