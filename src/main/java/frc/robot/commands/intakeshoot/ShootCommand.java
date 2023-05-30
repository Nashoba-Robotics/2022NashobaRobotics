package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class ShootCommand extends CommandBase {

    private boolean shooting;
    private double lastValidTy;
    private Angle angle;

    public ShootCommand(boolean shooting) {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        addRequirements(CannonSubsystem.getInstance());
        this.shooting = shooting;
        lastValidTy = 0;
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
        LoaderSubsystem.getInstance().set(0);
    }

    @Override
    public void execute() {
        // if(Math.abs(LimelightSubsystem.getInstance().getShooterTx()) <= Constants.AUTO_AIM_DEADZONE) {
        //     SmartDashboard.putBoolean("I've Got'chu in my Sights", true);
        // } else {
        //     SmartDashboard.putBoolean("I've Got'chu in my Sights", false);
        // }

        double cannonSpeed;
        angle = CannonSubsystem.getInstance().getAngle();
        
        if(angle == Angle.EIGHTY) {
            cannonSpeed = Constants.Cannon.CLOSE_SHOT_SPEED;
        } else {
            if(LimelightSubsystem.getInstance().shooterValidTarget()){
                lastValidTy = LimelightSubsystem.getInstance().getShooterTy();
            }
            cannonSpeed = Constants.Cannon.farShotSpeed(lastValidTy);

        }
        double loaderSpeed = angle == Angle.EIGHTY ? 0.55 : 0.7; //0.4  0.5
        CannonSubsystem.getInstance().set(cannonSpeed);
        if(shooting) {
            LoaderSubsystem.getInstance().set(loaderSpeed);
        } else {
            LoaderSubsystem.getInstance().set(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
        LoaderSubsystem.getInstance().set(0);
    }
}
