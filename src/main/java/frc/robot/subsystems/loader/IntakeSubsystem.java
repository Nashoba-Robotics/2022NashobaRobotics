package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intake;
    private static IntakeSubsystem singleton;

    public IntakeSubsystem(){
        intake = new TalonFX(Constants.Loader.INTAKE_PORT);
    }
    
    public static IntakeSubsystem getInstance(){
        if(singleton == null) singleton = new IntakeSubsystem();
        return singleton;
    }
    
    public void intake(){
        intake.set(ControlMode.PercentOutput, 0.3);
    }

    public void stop() {
        intake.set(ControlMode.PercentOutput, 0);
    }

    public void set(double speed) {
        intake.set(ControlMode.PercentOutput, speed);
    }
}
