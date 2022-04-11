package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class TestAutoAimCommand extends CommandBase{
    private Timer timer;
    private double leftTargetPos;
    private double rightTargetPos;

    public TestAutoAimCommand(){
        timer = new Timer();
        timer.start();
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(GyroSubsystem.getInstance());
        // SmartDashboard.putNumber("auto angle", 0);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Left stator", 0);
        SmartDashboard.putNumber("Right stator", 0);

        GyroSubsystem.getInstance().zeroHeading();
        double angle = -LimelightSubsystem.getInstance().getShooterTx();
        double[] targetPoses = DriveSubsystem.getInstance().turnToAngle(angle + Math.signum(angle) * 3);
        leftTargetPos = targetPoses[0];
        rightTargetPos= targetPoses[1];
        timer.reset();

        SmartDashboard.putBoolean("Aim Start", true);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().setSpeed(0);
        SmartDashboard.putBoolean("Aim Start", false);
    }

    @Override
    public boolean isFinished() {
        double leftDifference = leftTargetPos - DriveSubsystem.getInstance().getPositionLeft();
        double rightDifference = rightTargetPos - DriveSubsystem.getInstance().getPositionRight();
        
        return (Math.abs(leftDifference) < 1150
            && Math.abs(rightDifference) < 1000)
            || timer.get() >= 1
            ;
    }
}
