package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends CommandBase {
    public RunIntakeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        boolean ball1 = IntakeSubsystem.getInstance().getSensor2(); 
        boolean ball2 = ball1 && IntakeSubsystem.getInstance().getSensor2(); 
        IntakeSubsystem.getInstance()
            .setIntake(ball2 ? 0 : 0.5)
            .setGrabber(ball2 ? 0 : 0.4)
            .setLoader(ball1 ? 0 : 0.3);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
