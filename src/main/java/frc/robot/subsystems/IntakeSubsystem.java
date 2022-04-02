package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX motor;

    private IntakeSubsystem(){
        motor = new TalonFX(Constants.Intake.PORT_INTAKE);
    }
    
    private static IntakeSubsystem instance;
    public static IntakeSubsystem getInstance(){
        if(instance == null) instance = new IntakeSubsystem();
        return instance;
    }    

    public IntakeSubsystem set(double speed){
        motor.set(ControlMode.PercentOutput, speed);
        return this;
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public double getCurrent() {
        return motor.getStatorCurrent();
    }
}
