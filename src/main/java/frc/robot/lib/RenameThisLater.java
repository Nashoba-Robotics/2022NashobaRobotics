package frc.robot.lib;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;

public class RenameThisLater {
    private double startValue, stopValue, time;
    private Timer timer;

    public RenameThisLater(double startValue, double stopValue, double time) {
        this.startValue = startValue;
        this.stopValue = stopValue;
        this.time = time;
        timer = new Timer();
    }

    public void start() {
        timer.start();
    }

    public double get() {
        if(timer.get() >= time) {
            return stopValue;
        }
        return startValue + (stopValue - startValue)*(timer.get()/time);
    }

    public void restart(double stopValue, double time) {
        this.startValue = get();
        this.stopValue = stopValue;
        this.time = time;
        timer.reset();
    }

    public void setStopValue(double stopValue){
        this.stopValue = stopValue;
    }

    public boolean finished() {
        return timer.get() >= time;
    }
}
