package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intake;
    private TalonFX grabber;
    private TalonFX loader;

    private DigitalInput sensor1;
    private DigitalInput sensor2;

    private static IntakeSubsystem singleton;

    public IntakeSubsystem(){
        intake = new TalonFX(Constants.Intake.PORT_INTAKE);
        grabber = new TalonFX(Constants.Intake.PORT_GRABBER);
        loader = new TalonFX(Constants.Intake.PORT_LOADER);
        sensor1 = new DigitalInput(Constants.Intake.DIO_SENSOR_1);
        sensor2 = new DigitalInput(Constants.Intake.DIO_SENSOR_2);
    }
    
    public static IntakeSubsystem getInstance(){
        if(singleton == null) singleton = new IntakeSubsystem();
        return singleton;
    }
    
    public IntakeSubsystem setIntake(double speed){
        intake.set(ControlMode.PercentOutput, speed);
        return this;
    }
        
    public IntakeSubsystem setGrabber(double speed){
        grabber.set(ControlMode.PercentOutput, speed);
        return this;
    }
        
    public IntakeSubsystem setLoader(double speed){
        loader.set(ControlMode.PercentOutput, speed);
        return this;
    }

    public boolean getSensor1() { return sensor1.get(); }
    public boolean getSensor2() { return sensor2.get(); }

    public void stop() {
        intake.set(ControlMode.PercentOutput, 0);
        grabber.set(ControlMode.PercentOutput, 0);
        loader.set(ControlMode.PercentOutput, 0);
    }
}
