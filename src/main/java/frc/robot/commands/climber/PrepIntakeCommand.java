package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PrepIntakeCommand extends CommandBase{

    Timer timer;
    boolean finished;

    public PrepIntakeCommand(){
        timer = new Timer();
    }

    @Override
    public void initialize() {
        finished = IntakeSolenoidSubsystem.getInstance().getState();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() > 0.5){
            finished = true;
        }
    }


    @Override
    public boolean isFinished() {
        return IntakeSolenoidSubsystem.getInstance().getState()
            && finished;
    }
}
