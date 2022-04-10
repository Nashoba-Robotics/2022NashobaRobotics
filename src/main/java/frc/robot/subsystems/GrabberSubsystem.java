package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase  {
    private TalonFX motor;

    private GrabberSubsystem() {
        motor = new TalonFX(Constants.Intake.PORT_GRABBER);
        motor.setInverted(true);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    }

    private static GrabberSubsystem instance;
    public static GrabberSubsystem getInstance(){
        if(instance == null) instance = new GrabberSubsystem();
        return instance;
    }

    public void set(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
    
    public double getPercentOutput(){
        return motor.getMotorOutputPercent();
    }
}
