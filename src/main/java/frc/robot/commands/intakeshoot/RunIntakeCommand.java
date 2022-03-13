package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class RunIntakeCommand extends CommandBase {
    public RunIntakeCommand() {
        this.
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Intake On?", true);
        SmartDashboard.putNumber("Balls", 0);
    }

    @Override
    public void execute() { 
        boolean ball1 = RobotContainer.getSensor2(); 
        boolean ball2 = ball1 && RobotContainer.getSensor1(); 
        IntakeSubsystem.getInstance().set(ball2 ? -0.2 : Constants.Intake.INTAKE_SPEED);
        GrabberSubsystem.getInstance().set(ball2 ? 0 : Constants.Intake.GRABBER_SPEED);
        LoaderSubsystem.getInstance().set(ball1 ? 0 : Constants.Intake.LOADER_SPEED);
        int balls = (ball1 ? 1 : 0) + (ball2 ? 1 : 0);
        SmartDashboard.putNumber("Balls", balls);

    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
        LoaderSubsystem.getInstance().stop();
        SmartDashboard.putBoolean("Intake On?", false);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
