package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MusicSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class OdeToJoy extends CommandBase{
    Timer timer = new Timer();
    final double G = 392, A = 440, B = 494, C = 523, D = 587, E = 659, F = 698, G2 = 784;
    //Each letter is an eigth note
    final double[] song = {
        C,C, 0, C,C, 0, D,D, 0, E,E, 0, E,E, 0, D,D, 0, C,C, 0, B,B, 0, A,A, 0, A,A, 0, B,B, 0, C,C, 0, C,C,C, 0, B, 0, B,B,B,B, 0,
        C,C, 0, C,C, 0, D,D, 0, E,E, 0, E,E, 0, D,D, 0, C,C, 0, B,B, 0, A,A, 0, A,A, 0, B,B, 0, C,C, 0, B,B,B, 0, A, 0, A,A,A,A, 0,
        D,D, 0, D,D, 0, E,E, 0, C,C, 0, D,D, 0, E, 0, F, 0, E,E, 0, C,C, 0, D,D, 0, E, 0, F, 0, E,E, 0, D,D, 0, C,C, D,D, 0, G,G,G,G, 0,
        E,E, 0, E,E, 0, F,F, 0, G2,G2, 0, G2,G2, 0, F,F, 0, E,E, 0, D,D, 0, C,C, 0, C,C, 0, D,D, 0, E,E, 0, D,D,D, 0, C, 0, C,C,C,C, 0
    };
    int i = 0;
    boolean finished = false;

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        MusicSubsystem.getInstance().setFrequency(song[i]);
        if(i >= song.length) finished = true;
        else if(timer.get() > 0.25){
            timer.reset();
            i++;
            if(song[i] == 0){
                MusicSubsystem.getInstance().setFrequency(0);
                i++;
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        MusicSubsystem.getInstance().setFrequency(0);
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}