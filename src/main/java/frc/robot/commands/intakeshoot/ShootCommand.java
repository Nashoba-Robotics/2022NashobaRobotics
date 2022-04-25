package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class ShootCommand extends CommandBase {
    public static final Trigger ANGLE = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 7).debounce(0.1);

    private boolean shooting;
    private double lastValidTy;

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

        Angle angle = ANGLE.get() ? Angle.EIGHTY : Angle.SIXTY;
        double cannonSpeed;
        if(angle == Angle.EIGHTY) {
            cannonSpeed = Constants.Cannon.CLOSE_SHOT_SPEED;
        } else {
            if(LimelightSubsystem.getInstance().shooterValidTarget()){
                lastValidTy = LimelightSubsystem.getInstance().getShooterTy();
            }
            cannonSpeed = Constants.Cannon.farShotSpeed(lastValidTy);
        }
        double loaderSpeed = angle == Angle.EIGHTY ? 0.4 : 0.5;
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
