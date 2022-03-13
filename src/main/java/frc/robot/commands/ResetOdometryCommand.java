package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class ResetOdometryCommand extends CommandBase{
    Trajectory trajectory;
    public ResetOdometryCommand(Trajectory trajectory){
        this.trajectory = trajectory;
    }
    @Override
    public void execute() {
        GyroSubsystem.getInstance();
        DriveSubsystem.getInstance().resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
