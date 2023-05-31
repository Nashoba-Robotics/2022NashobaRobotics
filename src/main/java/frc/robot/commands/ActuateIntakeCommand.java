package frc.robot.commands;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ActuateIntakeCommand extends CommandBase {
    private boolean state;
    private Timer timer;

    public ActuateIntakeCommand(boolean state) {
        timer = new Timer();
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        this.state = state;
    }

    @Override
    public void initialize() {
        IntakeSolenoidSubsystem.getInstance().setState(state);
        if(!state){
            timer.reset();
            timer.start();
        }
    }

    @Override
    public void execute() {
        if(!state){
            GrabberSubsystem.getInstance().set(Constants.Intake.GRABBER_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().set(0);
    }
    
    @Override
    public boolean isFinished() {
       return state || timer.get() > 0.5;
    }
}  
