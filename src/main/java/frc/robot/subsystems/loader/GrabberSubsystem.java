package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.AbstractDriveSubsystem;

public class GrabberSubsystem extends SubsystemBase{
    private TalonFX grabber;
    private static GrabberSubsystem singleton;

    public GrabberSubsystem(){
        grabber = new TalonFX(Constants.Loader.GRABBER_PORT);
    }

    public static GrabberSubsystem getInstance(){
        if(singleton == null) singleton = new GrabberSubsystem();
        return singleton;
    }

    public void set(double speed) {
        grabber.set(ControlMode.PercentOutput, speed);
    }

    public void intake(){
    }

    public void puke(){

    }
}
