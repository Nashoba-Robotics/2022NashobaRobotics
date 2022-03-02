package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ShootCommand extends CommandBase {
    long startMillis;
    boolean onLast;

    public ShootCommand() {
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(CannonSubsystem.getInstance());
        addRequirements(LimelightSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        startMillis = System.currentTimeMillis();
        SmartDashboard.putNumber("Shooter", 0);
        SmartDashboard.putNumber("On", 0);
        onLast = false;
    }

    @Override
    public void execute() { 
        //double ty = LimelightSubsystem.getInstance().getShooterTy();
        //double speed = 0.815 - 0.00951*ty;


        boolean on = SmartDashboard.getNumber("On", 0) != 0;

        if(on && !onLast) {
            startMillis = System.currentTimeMillis();
        }

        onLast = on;

        double speed = SmartDashboard.getNumber("Shooter", 0);

        if(on) {
            CannonSubsystem.getInstance().set(speed);    

            if(System.currentTimeMillis() > startMillis + 300) {
                IntakeSubsystem.getInstance()
                .setIntake(0)
                .setGrabber(0)
                .setLoader(0.5);
                // 0.5 far away, 0.4 up close
            } else {
                IntakeSubsystem.getInstance()
                .setIntake(0)
                .setGrabber(0)
                .setLoader(0);
            }
        } else {
            IntakeSubsystem.getInstance().stop();
            CannonSubsystem.getInstance().set(0);
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().stop();
        CannonSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}  
