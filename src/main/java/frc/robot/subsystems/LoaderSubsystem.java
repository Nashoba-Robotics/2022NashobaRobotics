package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LoaderSubsystem extends SubsystemBase {
    private TalonFX motor;

    private LoaderSubsystem() {
        motor = new TalonFX(Constants.Intake.PORT_LOADER);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    }

    private static LoaderSubsystem instance;
    public static LoaderSubsystem getInstance(){
        if(instance == null) instance = new LoaderSubsystem();
        return instance;
    }

    public void set(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}