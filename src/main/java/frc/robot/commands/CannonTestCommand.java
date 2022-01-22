package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;

public class CannonTestCommand extends CommandBase {
    double speeeeeeed;
    public CannonTestCommand(){
        addRequirements(CannonSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        speeeeeeed = SmartDashboard.getNumber("Cannon Speed", 0);
        SmartDashboard.putNumber("Cannon Speed", speeeeeeed);
    }

    @Override
    public void execute(){
        CannonSubsystem.getInstance().shoot(speeeeeeed);
        
        speeeeeeed = SmartDashboard.getNumber("Cannon Speed", 0);
        SmartDashboard.putNumber("Top speed", CannonSubsystem.getInstance().getTopSpeed());
        SmartDashboard.putNumber("Bottom speed", CannonSubsystem.getInstance().getBottomSpeed());
    }

    @Override
    public void end(boolean interrupted){
        CannonSubsystem.getInstance().shoot(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}