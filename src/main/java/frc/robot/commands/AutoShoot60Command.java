package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShoot60Command extends CommandBase{
    public AutoShoot60Command(){
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(IntakeSubsystem.getInstance());
    }

    double startMillis;

    @Override
    public void initialize() {
        startMillis = System.currentTimeMillis();
        CannonSubsystem.getInstance().setSolenoid(true);
    }

    public void execute() {
        long millis = System.currentTimeMillis();
        
        CannonSubsystem.getInstance().set(0.5);

        if(millis > startMillis + 300) {
            IntakeSubsystem.getInstance()
            .setIntake(0)
            .setGrabber(0)
            .setLoader(0.4);
        }
    }

    public boolean isFinished() {
        return System.currentTimeMillis() > startMillis + 2350;
    }

    @Override
    public void end(boolean interrupted) {
        CannonSubsystem.getInstance().set(0);
        IntakeSubsystem.getInstance().stop();
    }
}
