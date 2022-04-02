package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetColorRejectionCommand extends CommandBase {
    boolean on;

    public SetColorRejectionCommand(boolean on) {
        this.on = on;
    }
    
    @Override
    public void execute() {
        RunIntakeCommand.setColorRejection(on);
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}  
