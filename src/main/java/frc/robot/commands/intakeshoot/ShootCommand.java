package frc.robot.commands.intakeshoot;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class ShootCommand extends CommandBase {
    long startMillis;
    long stopMillis;
    boolean on;
    boolean finished;

    double lastValidTy = 0;

    Trigger shoot = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 9).debounce(0.1);
    Trigger angle = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 7).debounce(0.1);

    public ShootCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(GrabberSubsystem.getInstance());
        addRequirements(LoaderSubsystem.getInstance());
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(LimelightSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        startMillis = System.currentTimeMillis();
        stopMillis = System.currentTimeMillis();
        on = false;
        finished = false;
        SmartDashboard.putBoolean("Shooter On?", true);
    }

    @Override
    public void execute() {         
        long millis = System.currentTimeMillis();

        if(Math.abs(LimelightSubsystem.getInstance().getIntakeTx()) <= Constants.AUTO_AIM_DEADZONE)
        SmartDashboard.putBoolean("I've Got'chu in my Sights", true);
        else SmartDashboard.putBoolean("I've Got'chu in my Sights", false);

        if(shoot.get()) {
            on = true;
            startMillis = millis;
            stopMillis = Long.MAX_VALUE;
        }

        boolean eightydeg = angle.get();
        double cannonSpeed;

        if(!eightydeg) {
            if(LimelightSubsystem.getInstance().shooterValidTarget()){
                lastValidTy = LimelightSubsystem.getInstance().getShooterTy();
            }
            cannonSpeed = 0.53 - 0.00825 * lastValidTy;
            //cannonSpeed = 0.545 - 0.007 * lastValidTy;
        } else {
            cannonSpeed = 0.465; // close up shot
        }

        CannonSubsystem.getInstance().set(cannonSpeed);    

        if(on) {
            double loaderSpeed = eightydeg ? 0.4 : 0.5;
            IntakeSubsystem.getInstance().set(0);
            GrabberSubsystem.getInstance().set(0);
            LoaderSubsystem.getInstance().set(loaderSpeed);
        } else {
            IntakeSubsystem.getInstance().stop();
        }

        if(millis < stopMillis && !RobotContainer.getSensor1() && !RobotContainer.getSensor2()) {
            stopMillis = millis;
        }

        // TODO uncomment
        // if(millis > stopMillis + 500) {
        //     finished = true;
        //     SmartDashboard.putString("Logging", "finished");
        // }
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        CannonSubsystem.getInstance().set(0);
        LoaderSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().set(0);
        SmartDashboard.putBoolean("Shooter On?", false);
    }

    @Override
    public boolean isFinished() {
       return finished;
    }
}  
