package frc.robot.commands.autoroutines;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.StopCommand;
import frc.robot.subsystems.DriveSubsystem;

public class PathFollowCommand extends SequentialCommandGroup{
    Trajectory trajectory;
    public PathFollowCommand(String trajectoryJSON){
        trajectory = new Trajectory();
    
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
       } catch (IOException e) {
          DriverStation.reportError("Unable to open trajectory: \n \n \n \n", e.getStackTrace());
       }

       RamseteCommand ramseteCommand =
       new RamseteCommand(
           trajectory,
           DriveSubsystem.getInstance()::getPose,
           new RamseteController(Constants.DriveTrain.AUTO_B, Constants.DriveTrain.AUTO_ZETA),
           DriveSubsystem.getInstance().getKinematics(),
           DriveSubsystem.getInstance()::setMetersPerSecond,
           DriveSubsystem.getInstance());


       addCommands(
        new ResetOdometryCommand(trajectory),
        ramseteCommand,
        new StopCommand()
       );
    }

    public PathFollowCommand(Trajectory trajectory){
        RamseteCommand ramseteCommand =
       new RamseteCommand(
           trajectory,
           DriveSubsystem.getInstance()::getPose,
           new RamseteController(Constants.DriveTrain.AUTO_B, Constants.DriveTrain.AUTO_ZETA),
           DriveSubsystem.getInstance().getKinematics(),
           DriveSubsystem.getInstance()::setMetersPerSecond,
           DriveSubsystem.getInstance());


       addCommands(
        new ResetOdometryCommand(trajectory),
        ramseteCommand,
        new StopCommand()
       );
    }
}
