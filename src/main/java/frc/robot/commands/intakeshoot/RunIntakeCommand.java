package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends CommandBase {
    public RunIntakeCommand() {
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Intake On?", true);
        SmartDashboard.putNumber("Balls", 0);
    }

    @Override
    public void execute() { 
        boolean ball1 = IntakeSubsystem.getInstance().getSensor2(); 
        boolean ball2 = ball1 && IntakeSubsystem.getInstance().getSensor1(); 
        IntakeSubsystem.getInstance()
            .setIntake(ball2 ? 0 : 0.75)
            .setGrabber(ball2 ? 0 : 0.4)
            .setLoader(ball1 ? 0 : 0.15);
        int balls = (ball1 ? 1 : 0) + (ball2 ? 1 : 0);
        SmartDashboard.putNumber("Balls", balls);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        SmartDashboard.putBoolean("Intake On?", false);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
