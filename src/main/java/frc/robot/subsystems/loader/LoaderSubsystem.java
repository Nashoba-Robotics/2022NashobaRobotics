package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LoaderSubsystem extends SubsystemBase{
    private TalonFX loader;
    private static LoaderSubsystem singleton;

    public LoaderSubsystem(){
        loader = new TalonFX(Constants.Loader.LOADER_PORT);
    }

    public static LoaderSubsystem getInstance(){
        if(singleton == null) singleton = new LoaderSubsystem();
        return singleton;
    }

    public void intake(){
        loader.set(ControlMode.PercentOutput, 0.3);
    }

    public void stop() {
        loader.set(ControlMode.PercentOutput, 0);
    }

    public void set(double speed) {
        loader.set(ControlMode.PercentOutput, speed);
    }
}
