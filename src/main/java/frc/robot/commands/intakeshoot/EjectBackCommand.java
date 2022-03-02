package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectBackCommand extends CommandBase {
    public EjectBackCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(CannonSubsystem.getInstance());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        boolean sensor1 = IntakeSubsystem.getInstance().getSensor1();
        IntakeSubsystem.getInstance()
            .setIntake(0)
            .setGrabber(0)
            .setLoader(sensor1 ? 0.2 : 0);
        CannonSubsystem.getInstance().set(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        CannonSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
