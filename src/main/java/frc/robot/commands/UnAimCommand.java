package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class UnAimCommand extends CommandBase{
    Timer timer;
    
    public UnAimCommand(){
        timer = new Timer();
        addRequirements(DriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        SmartDashboard.putNumber("Return Angle", DriveSubsystem.getInstance().getReturnAngle());
        DriveSubsystem.getInstance().unAim();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        double leftDifference = DriveSubsystem.getInstance().getReturnPos()[0] - DriveSubsystem.getInstance().getPositionLeft();
        double rightDifference = DriveSubsystem.getInstance().getReturnPos()[1]- DriveSubsystem.getInstance().getPositionRight();
        
        return (Math.abs(leftDifference) < 1150
            && Math.abs(rightDifference) < 1000)
            || timer.get() >= 0.3;
    }
    
}
