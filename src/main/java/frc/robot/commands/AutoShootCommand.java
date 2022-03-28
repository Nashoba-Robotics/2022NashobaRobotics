package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Cannon;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class AutoShootCommand extends CommandBase{
    long startMillis;
    Angle angle;
    double cannonSpeed;
    double lastValidTy;

    public AutoShootCommand(Angle angle) {
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        
        this.angle = angle;
    }

    @Override
    public void initialize() {
        startMillis = System.currentTimeMillis();
        CannonSubsystem.getInstance().setAngle(angle);

        cannonSpeed = 0;
        lastValidTy = 0;
    }

    @Override
    public void execute() {
        long millis = System.currentTimeMillis();
        
        if(angle == Angle.SIXTY) {
            if(LimelightSubsystem.getInstance().shooterValidTarget()){
                lastValidTy = LimelightSubsystem.getInstance().getShooterTy();
            }
            cannonSpeed = 0.53 - 0.00825 * lastValidTy;
            //cannonSpeed = 0.545 - 0.007 * lastValidTy;
        } else {
            cannonSpeed = 0.465; // close up shot
        }

        CannonSubsystem.getInstance().set(cannonSpeed);

        if(millis > startMillis + 300) {
            IntakeSubsystem.getInstance().set(0);
            GrabberSubsystem.getInstance().set(Constants.Intake.GRABBER_SPEED);
            LoaderSubsystem.getInstance().set(angle == Angle.EIGHTY ? 0.4 : 0.5);
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > startMillis + 1000;
    }

    @Override
    public void end(boolean interrupted) {
        CannonSubsystem.getInstance().set(0);
        IntakeSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
        LoaderSubsystem.getInstance().stop();
    }
    
}
