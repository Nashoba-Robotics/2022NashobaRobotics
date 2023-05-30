package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Cannon;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class AutoAimMotionMagicCommand extends CommandBase{
    private Timer timer;
    private double leftTargetPos;
    private double rightTargetPos;
    private boolean isShooting;
    private boolean aimWhileShooting;

    public AutoAimMotionMagicCommand(boolean isShooting){
        timer = new Timer();
        timer.start();
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(GyroSubsystem.getInstance());
        //addRequirements(CannonSubsystem.getInstance());
        this.isShooting = isShooting;
    }

    @Override
    public void initialize() {
        // if(aimWhileShooting || !isShooting){
            // GyroSubsystem.getInstance().zeroHeading();
            double angle = -LimelightSubsystem.getInstance().getShooterTx();
            System.out.println(angle);
            double[] targetPoses = DriveSubsystem.getInstance().turnToAngle(angle + Math.signum(angle) * Constants.Limelight.AUTO_AIM_OFFSET);
            leftTargetPos = targetPoses[0];
            rightTargetPos= targetPoses[1];
        // }
        // SmartDashboard.putBoolean("Shooting?", isShooting);
        aimWhileShooting = DriveSubsystem.getInstance().getDoAim();
        // SmartDashboard.putBoolean("Aim?", aimWhileShooting);
        // SmartDashboard.putNumber("Return Angle", DriveSubsystem.getInstance().getReturnAngle());
        timer.reset();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if(!DriverStation.isAutonomous() && CannonSubsystem.getInstance().getAngle() == Angle.EIGHTY) {
            return true;
        }
        if(!LimelightSubsystem.getInstance().shooterValidTarget()){
            return true;
        }
        double leftDifference = leftTargetPos - DriveSubsystem.getInstance().getPositionLeft();
        double rightDifference = rightTargetPos - DriveSubsystem.getInstance().getPositionRight();

        aimWhileShooting = DriveSubsystem.getInstance().getDoAim();
        
        return (Math.abs(leftDifference) < 1250
            && Math.abs(rightDifference) < 1100)
            // && (aimWhileShooting || !isShooting)
            || timer.get() >= 1;
    }
}
