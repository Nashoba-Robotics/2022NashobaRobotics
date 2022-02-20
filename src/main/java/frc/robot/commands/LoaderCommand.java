package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Loader;
import frc.robot.subsystems.loader.GrabberSubsystem;
import frc.robot.subsystems.loader.IntakeSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class LoaderCommand extends CommandBase  {
    public LoaderCommand() {
        addRequirements(LoaderSubsystem.getInstance());
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Loader", 0);
        SmartDashboard.putNumber("Intake", 0);
        SmartDashboard.putNumber("Grabber", 0);
    }

    @Override
    public void execute() {
        LoaderSubsystem.getInstance().set(SmartDashboard.getNumber("Loader",0));
        IntakeSubsystem.getInstance().set(SmartDashboard.getNumber("Intake",0));
        GrabberSubsystem.getInstance().set(SmartDashboard.getNumber("Grabber",0));
    }

    @Override
    public void end(boolean isInterrupted) {
        LoaderSubsystem.getInstance().set(0);
        IntakeSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
    }


}
