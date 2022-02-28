package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TristanCommand extends CommandBase{
    public TristanCommand(){

    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Tristan?", true);
    }

    @Override
    public void end(boolean inerrupted){
        SmartDashboard.putBoolean("Tristan?", false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
