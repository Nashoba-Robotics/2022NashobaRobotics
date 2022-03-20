package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private static final int LIGHT_COUNT = 100;
    private static final double BLINK_RATE = 2;

    private static LedSubsystem instance;
    public static LedSubsystem getInstance() {
        if(instance == null) {
            instance = new LedSubsystem();
        }
        return instance;
    }


    private CANdle candle;
    private int lastR = -1;
    private int lastG = -1;
    private int lastB = -1;
    private int lastMode = -1;

    private LedSubsystem() {
        candle = new CANdle(0);
        candle.configFactoryDefault();
        candle.configLOSBehavior(true);
        candle.configStatusLedState(true);
    }

    public void setColor(int r, int g, int b) {
        if(lastR != r || lastG != g || lastB != b || lastMode != 0) {
            candle.setLEDs(r, g, b, 0, 8, LIGHT_COUNT);
            lastR = r;
            lastG = g;
            lastB = b;
            lastMode = 0;
        }
    }

    public void blinkColor(int r, int g, int b) {
        if(lastR != r || lastG != g || lastB != b || lastMode != 1) {
            Animation a = new StrobeAnimation(r, g, b, 0, BLINK_RATE, LIGHT_COUNT + 8);
            candle.animate(a);
            // candle.setLEDs(0, 0, 0, 0, 0, 8);
            lastR = r;
            lastG = g;
            lastB = b;
            lastMode = 1;
        }
    }
    
}
