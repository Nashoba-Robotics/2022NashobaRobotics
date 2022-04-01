package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAimMotionMagicCommand extends CommandBase{
//    private double angle;
//    Timer timer;

    public AutoAimMotionMagicCommand(){
        // timer = new Timer();
        // timer.start();
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(GyroSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GyroSubsystem.getInstance().zeroHeading();
        // double angle = SmartDashboard.getNumber("Target Angle", 0);
        double angle = -LimelightSubsystem.getInstance().getIntakeTx();
        DriveSubsystem.getInstance().turnToAngle(angle + Math.signum(angle) * 3);
        // timer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().setSpeed(0);
        // SmartDashboard.putNumber("Aim Time", timer.get());
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(DriveSubsystem.getInstance().getPositionLeft() - DriveSubsystem.getInstance().getLeftTargetAimPos(angle)) <= 300 || 
        // Math.abs(DriveSubsystem.getInstance().getPositionRight() - DriveSubsystem.getInstance().getRightTargetAimPos(angle)) <= 300;  //<-- TEMPORARY
        return false;
    }
}
