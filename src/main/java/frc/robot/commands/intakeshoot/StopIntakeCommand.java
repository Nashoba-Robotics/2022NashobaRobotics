package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends CommandBase{
    public StopIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
