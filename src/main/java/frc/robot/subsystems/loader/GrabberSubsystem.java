package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase{
    private TalonFX grabber;
    private static GrabberSubsystem singleton;

    public GrabberSubsystem(){
        grabber = new TalonFX(Constants.Loader.GRABBER_PORT);
        grabber.setInverted(true);
    }

    public static GrabberSubsystem getInstance(){
        if(singleton == null) singleton = new GrabberSubsystem();
        return singleton;
    }
 
    public void intake(){
        grabber.set(ControlMode.PercentOutput, 0.3);
    }

    public void stop() {
        grabber.set(ControlMode.PercentOutput, 0);
    }

    public void set(double speed) {
        grabber.set(ControlMode.PercentOutput, speed);
    }
}
