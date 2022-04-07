package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleAutoAimCommand extends CommandBase{
    private boolean doAim;
    public ToggleAutoAimCommand(boolean doAim){
        this.doAim = doAim;
        addRequirements(DriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        DriveSubsystem.getInstance().changeAim(doAim);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
