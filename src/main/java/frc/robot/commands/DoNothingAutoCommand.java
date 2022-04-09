package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.LedStateType;

public class DoNothingAutoCommand extends CommandBase{

    @Override
    public void initialize() {
        LedSubsystem.getInstance().setLedStateType(LedStateType.GRACIOUS_PROFESSIONALISM);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
