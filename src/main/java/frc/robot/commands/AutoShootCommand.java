package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class AutoShootCommand extends CommandBase{
    long startMillis;
    boolean firstCycle;
    Angle angle;

    public AutoShootCommand(Angle angle) {
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        firstCycle = true;
        this.angle = angle;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(firstCycle) {
            startMillis = System.currentTimeMillis();
            CannonSubsystem.getInstance().setAngle(angle); 
            firstCycle = false;
        }
        long millis = System.currentTimeMillis();
        
        CannonSubsystem.getInstance().set(0.5);

        if(millis > startMillis + 300) {
            IntakeSubsystem.getInstance().set(0);
            GrabberSubsystem.getInstance().set(0);
            LoaderSubsystem.getInstance().set(0.4);
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > startMillis + 2350;
    }

    @Override
    public void end(boolean interrupted) {
        CannonSubsystem.getInstance().set(0);
        IntakeSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
        LoaderSubsystem.getInstance().stop();
        firstCycle = true;
    }
    
}
