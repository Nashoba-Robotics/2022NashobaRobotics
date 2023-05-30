package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class AutoShootCommand extends CommandBase{
    long startMillis;
    Angle angle;
    double speed;
    boolean manualSpeed;
    double cannonSpeed;
    double lastValidTy;

    public AutoShootCommand(Angle angle) {
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        
        this.angle = angle;
        manualSpeed = false;
    }

    public AutoShootCommand(Angle angle, double speed) {
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        
        this.angle = angle;
        this.speed = speed;
        manualSpeed = true;
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
            cannonSpeed = Constants.Cannon.farShotSpeed(lastValidTy);
        } else {
            cannonSpeed = Constants.Cannon.CLOSE_SHOT_SPEED;
        }

        if(manualSpeed) cannonSpeed = speed;

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
