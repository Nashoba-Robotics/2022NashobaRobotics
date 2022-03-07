package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends CommandBase{
    public StopIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setIntake(0);
        IntakeSubsystem.getInstance().setGrabber(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
