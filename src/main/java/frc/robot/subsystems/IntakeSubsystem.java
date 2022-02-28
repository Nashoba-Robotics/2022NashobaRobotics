package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intake;
    private TalonFX grabber;
    private TalonFX loader;

    private DigitalInput sensor1;
    private DigitalInput sensor2;

    private DoubleSolenoid intakeSolenoid;

    private static IntakeSubsystem singleton;

    public IntakeSubsystem(){
        intake = new TalonFX(Constants.Intake.PORT_INTAKE);
        grabber = new TalonFX(Constants.Intake.PORT_GRABBER);
        loader = new TalonFX(Constants.Intake.PORT_LOADER);
        sensor1 = new DigitalInput(Constants.Intake.DIO_SENSOR_1);
        sensor2 = new DigitalInput(Constants.Intake.DIO_SENSOR_2);

        grabber.setInverted(true);

        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Intake.INTAKE_SOLENOID_PORT, Constants.Intake.INTAKE_SOLENOID_PORT2);  //To be or not to be that is the question whether tis nobler in the mind to suffer the slings and arrows of outrageous fortunes or take arms against a sea of sorrows and by opposing end them to die to sleep no more and by a sleep to say we end the tousand natural shocks that flesh is heir to tis a consummation devoutly to be wished to die to sleep to sleep perchance to dream aye there's the rub for in that sleep of death what dreams may come when we have shuffled off this mortal coil must give us pause there's the respect that makes calamity of so long life for who would bear the whips and scorns of time the opressor's wrong the proud man's contumely the pangs of dispriz'd love the law's delay the insolence of office and the spurns that patient merit of the unworthy takes when he himself might his quietus make with a bare bodkin for who would fardels bear to grunt and sweat under a weary life but that the dread of something after death the undiscovered land from whose bourn no traveler returns puzzles the will and makes us rather bare those ills than fly to others that we know not of thus conscience doth make cowards of us all and thus the native hues of resolutions are sickled o'er with the pale cast of thought and enterprises of great pith and moment with this regard their currents turn awry and lose the name of actions  
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

    public IntakeSubsystem deployIntake() {
        if(intakeSolenoid.get() != Value.kReverse){
            intakeSolenoid.set(Value.kReverse); //Sets the solenoid back;
        }
        return this;
    }

    public IntakeSubsystem retractIntake() {    //Tristani es muy malo
        if(intakeSolenoid.get() != Value.kForward){
            intakeSolenoid.set(Value.kForward); //Sets the solenoid back;
        }
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
