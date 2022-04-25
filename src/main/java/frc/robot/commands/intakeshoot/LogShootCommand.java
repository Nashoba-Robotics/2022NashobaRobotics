package frc.robot.commands.intakeshoot;

import javax.naming.InitialContext;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LogShootCommand extends CommandBase{
    boolean shooting;
    public LogShootCommand(boolean b){
        shooting = b;
    }
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Started Shoot", shooting);
        System.out.println("Shoot Pressed: " + shooting);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
