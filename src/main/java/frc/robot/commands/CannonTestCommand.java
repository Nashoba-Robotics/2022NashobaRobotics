package frc.robot.commands;

import javax.swing.plaf.basic.BasicMenuUI.ChangeHandler;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Cannon;
import frc.robot.subsystems.CannonSubsystem;

/*
    Command to test the cannon. Allows the user to input a percent into Shuffleboard
    to use for the cannon velocity
*/
public class CannonTestCommand extends CommandBase {
    public CannonTestCommand(){
        addRequirements(CannonSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Cannon Speed", 0);
        SmartDashboard.putNumber("Solenoid", 0);
    }

    @Override
    public void execute(){

       double speed = SmartDashboard.getNumber("Cannon Speed", 0);
        CannonSubsystem.getInstance().set(speed);
        CannonSubsystem.getInstance().setAngle(SmartDashboard.getNumber("Solenoid", 0) != 0 ? CannonSubsystem.Angle.SIXTY : CannonSubsystem.Angle.EIGHTY);
        
        

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
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}