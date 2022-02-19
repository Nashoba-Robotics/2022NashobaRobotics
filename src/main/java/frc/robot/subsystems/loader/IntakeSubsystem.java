package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public IntakeSubsystem(){

    }
    
    private static IntakeSubsystem singleton;
    public static IntakeSubsystem getInstance(){
        if(singleton == null) singleton = new IntakeSubsystem();
        return singleton;
    }
    public void intake(){

    }

    public void puke(){

    }
}
