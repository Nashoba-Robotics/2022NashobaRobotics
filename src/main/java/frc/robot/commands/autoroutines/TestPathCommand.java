package frc.robot.commands.autoroutines;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetStartAngleCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TestPathCommand extends SequentialCommandGroup{
    public TestPathCommand(){

        DifferentialDriveVoltageConstraint autoVoltageConstraint = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);
    
        TrajectoryConfig config =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(autoVoltageConstraint);
    
        Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(5, 0, new Rotation2d(0)), //finishing position
          config);

        addCommands(
            new SetStartAngleCommand(Constants.TAU * 3 / 4),
            new PathFollowCommand(trajectory)
        );
    }
}
