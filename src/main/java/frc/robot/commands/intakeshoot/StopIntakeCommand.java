package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.lib.RenameThisLater;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class StopIntakeCommand extends CommandBase{
    private RenameThisLater thing;

    public StopIntakeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        thing = new RenameThisLater(GrabberSubsystem.getInstance().getPercentOutput(), 0, 0.3);
        thing.start();
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().set(0);
        LoaderSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(thing.get());
    }

    @Override
    public boolean isFinished() {
        return thing.finished();
    }
}
