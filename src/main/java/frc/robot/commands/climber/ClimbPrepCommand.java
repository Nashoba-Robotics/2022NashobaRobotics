package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ClimbPrepCommand extends CommandBase{

    Timer timer;
    boolean finished;

    public ClimbPrepCommand(){
        addRequirements(IntakeSolenoidSubsystem.getInstance());
        addRequirements(LimelightSubsystem.getInstance());
        timer = new Timer();
        timer.start();
    }

    @Override
    public void initialize() {
        finished = IntakeSolenoidSubsystem.getInstance().getState();
        timer.reset();
    }

    @Override
    public void execute() {
        IntakeSolenoidSubsystem.getInstance().deploy();
        LimelightSubsystem.getInstance().setShooterLed(1);
        if(timer.get() > 0.5){
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;

    }
}
