package frc.robot.commands.autoroutines;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
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
           Constants.DriveTrain.KINEMATICS,
           DriveSubsystem.getInstance()::setMetersPerSecond,
           DriveSubsystem.getInstance());


       addCommands(
        new ResetOdometryCommand(trajectory, Constants.FIELD.ANGLE_OF_RESISTANCE),
        ramseteCommand,
        new StopCommand()
       );
    }

    public PathFollowCommand(Trajectory trajectory){

        

        // TrajectoryConfig config = new TrajectoryConfig(
        //     Constants.DriveTrain.MAX_VELOCITY,
        //     Constants.DriveTrain.MAX_ACCELERATION);

        //config.setStartVelocity(0);
        // config.setReversed(false);
        // config.setEndVelocity(0);
        // config.addConstraint(new DifferentialDriveVoltageConstraint(
        //     new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
        //     Constants.DriveTrain.KINEMATICS,
        //     10));
        //config.addConstraint(new CentripetalAccelerationConstraint(Constants.DriveTrain.MAX_ACCELERATION));

        RamseteCommand ramseteCommand =
       new RamseteCommand(
           trajectory,
           DriveSubsystem.getInstance()::getPose,
           new RamseteController(Constants.DriveTrain.AUTO_B, Constants.DriveTrain.AUTO_ZETA),
           Constants.DriveTrain.KINEMATICS,
           DriveSubsystem.getInstance()::setMetersPerSecond,
           DriveSubsystem.getInstance());


       addCommands(
        new ResetOdometryCommand(trajectory, Constants.FIELD.ANGLE_OF_RESISTANCE),
        ramseteCommand,
        new StopCommand()
       );
    }
}
