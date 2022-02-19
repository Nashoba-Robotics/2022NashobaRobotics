package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoaderSubsystem extends SubsystemBase{
    public LoaderSubsystem(){

    }

    private static LoaderSubsystem singleton;
    public static LoaderSubsystem getInstance(){
        if(singleton == null) singleton = new LoaderSubsystem();
        return singleton;
    }
    public void transferBall(){

    }

    public void intake(){
        
    }

    public void puke(){
        
    }
}
