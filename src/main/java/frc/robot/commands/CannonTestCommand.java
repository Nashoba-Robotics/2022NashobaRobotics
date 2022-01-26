package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;

/*
    Command to test the cannon. Allows the user to input a percent into Shuffleboard
    to use for the cannon velocity
*/
public class CannonTestCommand extends CommandBase {
    double speed;
    public CannonTestCommand(){
        addRequirements(CannonSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        speed = SmartDashboard.getNumber("Cannon Speed", 0);
        SmartDashboard.putNumber("Cannon Speed", speed);
    }

    @Override
    public void execute(){
        CannonSubsystem.getInstance().shoot(speed);
        
        speed = SmartDashboard.getNumber("Cannon Speed", 0);
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