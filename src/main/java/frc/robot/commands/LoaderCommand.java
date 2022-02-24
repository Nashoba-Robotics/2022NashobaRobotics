package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
public class LoaderCommand extends CommandBase  {
    public LoaderCommand() {
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Loader", 0);
        SmartDashboard.putNumber("Intake", 0);
        SmartDashboard.putNumber("Grabber", 0);
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().setIntake(SmartDashboard.getNumber("Intake",0));
        IntakeSubsystem.getInstance().setGrabber(SmartDashboard.getNumber("Grabber",0));
        IntakeSubsystem.getInstance().setLoader(SmartDashboard.getNumber("Loader",0));
    }

    @Override
    public void end(boolean isInterrupted) {
        IntakeSubsystem.getInstance().stop();
    }


}
