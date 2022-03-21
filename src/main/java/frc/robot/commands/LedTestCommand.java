package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;

public class LedTestCommand extends CommandBase {
    public LedTestCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    public void initialize() {
        SmartDashboard.putNumber("R", 0);
        SmartDashboard.putNumber("G", 0);
        SmartDashboard.putNumber("B", 0);
    }

    public void execute() {
        int r = (int)SmartDashboard.getNumber("R", 0);
        int g = (int)SmartDashboard.getNumber("G", 0);
        int b = (int)SmartDashboard.getNumber("B", 0);
        LedSubsystem.getInstance().blinkColor(r, g, b);
    }
}
