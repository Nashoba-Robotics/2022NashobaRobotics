package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePracticeCommand extends CommandBase{
    public IntakePracticeCommand(){
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void execute() {
        IntakeSubsystem.getInstance().setIntake(0.75).setGrabber(0.4).setLoader(0.5);
        CannonSubsystem.getInstance().set(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        IntakeSubsystem.getInstance().setIntake(0).setGrabber(0).setLoader(0);
        CannonSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
