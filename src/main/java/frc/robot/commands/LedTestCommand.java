package frc.robot.commands;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;

public class LedTestCommand extends CommandBase {
    SendableChooser<Animation> animationChooser;
    
    public LedTestCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    public void initialize() {
        SmartDashboard.putNumber("R", 0);
        SmartDashboard.putNumber("G", 0);
        SmartDashboard.putNumber("B", 0);
    }

    public void execute() {
        int[] col = new int[]{
            (int)SmartDashboard.getNumber("R", 0),
            (int)SmartDashboard.getNumber("G", 0),
            (int)SmartDashboard.getNumber("B", 0),
        };
        LedSubsystem.getInstance().twinkle(col);
    }
}
