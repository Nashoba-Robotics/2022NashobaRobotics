package frc.robot.commands;

import java.beans.BeanInfo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericTalonSubsystem;

public class TalonTestCommand extends CommandBase{
    public TalonTestCommand(){
        addRequirements(GenericTalonSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Motor Position", 0);
    }

    @Override
    public void execute(){
        double pos = SmartDashboard.getNumber("Motor Position", 0);
        GenericTalonSubsystem.getInstance().getInstance().getInstance().getInstance().setPosition(pos);
    }

    @Override
    public void end(boolean interrupted){
        //GenericTalonSubsystem.getInstance().getInstance().getInstance().getInstance().setPosition(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
