package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ManualZeroOdometryCommand extends CommandBase{
    @Override
    public void execute() {
        DriveSubsystem.getInstance().resetOdometryTrue();
        DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
