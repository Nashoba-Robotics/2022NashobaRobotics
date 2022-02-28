package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake2021Subsystem extends SubsystemBase{
    VictorSPX intakeMotor;
    VictorSPX grabberMotor;
    public Intake2021Subsystem(){
        intakeMotor = new VictorSPX(6);
        grabberMotor = new VictorSPX(5);
    }

    private static Intake2021Subsystem singleton;
    public static Intake2021Subsystem getInstance(){
        if(singleton == null) singleton = new Intake2021Subsystem();
        return singleton;
    }

    public void intake(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
        grabberMotor.set(ControlMode.PercentOutput, speed);
    }
}
