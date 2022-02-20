package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LoaderSubsystem extends SubsystemBase{
    private TalonFX loader;

    public LoaderSubsystem(){
        loader = new TalonFX(Constants.Loader.LOADER_PORT);
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

    public void set(double speed) {
        loader.set(ControlMode.PercentOutput, speed);
    }
}
