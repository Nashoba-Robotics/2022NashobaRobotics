package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
public class LoaderCommand extends CommandBase  {
    public LoaderCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Loader", 0);
        SmartDashboard.putNumber("Intake", 0);
        SmartDashboard.putNumber("Grabber", 0);
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().set(SmartDashboard.getNumber("Intake",0));
        GrabberSubsystem.getInstance().set(SmartDashboard.getNumber("Grabber",0));
        LoaderSubsystem.getInstance().set(SmartDashboard.getNumber("Loader",0));
    }

    @Override
    public void end(boolean isInterrupted) {
        IntakeSubsystem.getInstance().stop();
    }


}
