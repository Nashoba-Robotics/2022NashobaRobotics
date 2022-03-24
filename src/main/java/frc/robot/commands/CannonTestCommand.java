package frc.robot.commands;

import javax.swing.plaf.basic.BasicMenuUI.ChangeHandler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Cannon;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

/*
    Command to test the cannon. Allows the user to input a percent into Shuffleboard
    to use for the cannon velocity
*/
public class CannonTestCommand extends CommandBase {
    SendableChooser<Angle> cannonAngle;
    boolean shooting = false;
    Timer timer;


    public CannonTestCommand(){
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        cannonAngle = new SendableChooser<>();
        cannonAngle.addOption("80", Angle.EIGHTY);
        cannonAngle.addOption("60", Angle.SIXTY);
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Cannon Speed", 0);
        SmartDashboard.putNumber("Loader speed", 0);
        SmartDashboard.putData(cannonAngle);
        timer = new Timer();
        shooting = false;
    }

    @Override
    public void execute(){
        double cannonSpeed = SmartDashboard.getNumber("Cannon Speed", 0);
        double loaderSpeed = SmartDashboard.getNumber("Loader speed", 0);
        CannonSubsystem.getInstance().setAngle(cannonAngle.getSelected());
        if(shooting) {
            GrabberSubsystem.getInstance().set(0);
            if(timer.get() > 1.7) {
                CannonSubsystem.getInstance().set(0);
                LoaderSubsystem.getInstance().set(0);
                shooting = false;
            } else if(timer.get() > 0.7) {
                LoaderSubsystem.getInstance().set(loaderSpeed);
                CannonSubsystem.getInstance().set(cannonSpeed);
            }
            else if(timer.get() > 0.2) {
                CannonSubsystem.getInstance().set(cannonSpeed);
                LoaderSubsystem.getInstance().set(0);
            } else {
                CannonSubsystem.getInstance().set(0);
                LoaderSubsystem.getInstance().set(0);
            }
        } else {
            if(RobotContainer.getSensor2()) {
                shooting = true;
                timer.reset();
                timer.start();
            } else {
                CannonSubsystem.getInstance().set(0);
                GrabberSubsystem.getInstance().set(Constants.Intake.GRABBER_SPEED);
                LoaderSubsystem.getInstance().set(Constants.Intake.LOADER_SPEED);
            }
        }

        //CannonSubsystem.getInstance().setAngle(SmartDashboard.getNumber("Solenoid", 0) != 0 ? CannonSubsystem.Angle.SIXTY : CannonSubsystem.Angle.EIGHTY);
        
        SmartDashboard.putNumber("topCurrent", CannonSubsystem.getInstance().getCurrentTop());
        SmartDashboard.putNumber("bottomCurrent", CannonSubsystem.getInstance().getCurretBottom());
        // CannonSubsystem.getInstance().setProportional(SmartDashboard.getNumber("Cannon P", 0));
        // CannonSubsystem.getInstance().setIntegral(SmartDashboard.getNumber("Cannon I", 0));
        // CannonSubsystem.getInstance().setDerivative(SmartDashboard.getNumber("Cannon D", 0));
        // CannonSubsystem.getInstance().setKF(SmartDashboard.getNumber("Cannon kF", 0));
    }

    @Override
    public void end(boolean interrupted){
        CannonSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
        LoaderSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}