package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Test1Command extends CommandBase{
    @Override
    public void execute() {
        SmartDashboard.putString("Test 2", "Ben");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
