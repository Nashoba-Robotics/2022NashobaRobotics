package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;

// Default command for LedSubsystem, use the ball sensors to set the LED colors
public class DefaultLedCommand extends CommandBase {
    public DefaultLedCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    public void initialize() {
        System.out.println("DefaultLedCommand initialized");
    }
    
    public void execute() {
        if(DriverStation.isTeleop()) {
            if(RobotContainer.getSensor1() && RobotContainer.getSensor2()){
                LedSubsystem.getInstance().setColor(255, 0, 0);
            } else if(RobotContainer.getSensor1() || RobotContainer.getSensor2()){
                LedSubsystem.getInstance().setColor(0, 0, 255);
            } else{
                LedSubsystem.getInstance().setColor(255, 255, 255);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DefaultLedCommand interrupted");
    }
}
