package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    public GrabberSubsystem(){

    }

    private static GrabberSubsystem singleton;
    public static GrabberSubsystem getInstance(){
        if(singleton == null) singleton = new GrabberSubsystem();
        return singleton;
    }
    public void intake(){

    }

    public void puke(){

    }
}
