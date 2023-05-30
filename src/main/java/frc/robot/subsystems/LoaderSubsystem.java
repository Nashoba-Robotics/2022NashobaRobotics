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

        // motor.config_kF(0, Constants.Intake.LOADER_KF);
        // motor.config_kP(0, Constants.Intake.LOADER_KP);
        // motor.config_kI(0, Constants.Intake.LOADER_KI);
        // motor.config_kD(0, Constants.Intake.LOADER_KD);
    }

    private static LoaderSubsystem instance;
    public static LoaderSubsystem getInstance(){
        if(instance == null) instance = new LoaderSubsystem();
        return instance;
    }

    public void set(double speed){
        // speed *= 22000;
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}